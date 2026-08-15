/*
  main.c - grblHAL-glowforge entry point

  Part of grblHAL-glowforge. Argument handling and the TCP listener are
  descended from the grblHAL Simulator's main.c (Copyright (c) 2012 Jens
  Geisler, 2014-2015 Adam Shelly, 2020 Terje Io). grbl_enter() runs on
  the main thread; the stepper stream owns its own threads
  (stepper_stream.c).

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

// grbl headers first: glibc's <sys/stat.h> (via fcntl.h) defines st_mtime
// as a macro, which must not be in scope when the core's vfs.h declares
// its struct field of the same name.
#include "build_info.h"
#include "driver.h"
#include "eeprom.h"
#include "platform.h"
#include "serial.h"

#include "grbl/grbllib.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

static const char *progname;

static void print_usage (const char *badarg)
{
    if(badarg)
        printf("Unrecognized option %s\n", badarg);

    printf("Usage:\n"
      "%s [options]\n"
      "  Options:\n"
      "    -p <port>        : TCP port for raw Grbl protocol (e.g. 23). Default: stdio.\n"
      "    -e <EEPROM file> : file holding grblHAL settings. default = EEPROM.DAT\n"
      "    -h               : this help.\n"
      "    -v, --version    : print version and build information.\n"
      "\n"
      "  Environment:\n"
      "    GFSINK           : pulse device (/dev/glowforge). Unset = null-sink mode.\n"
      "    GFSINK_RATE      : machine tick Hz (default 28160, the factory travel tick).\n"
      "    GFSINK_DEPTH_MS  : stream queue depth = feed-hold latency (default 200).\n"
      "    GFSINK_DUMP      : mirror the shipped pulse stream to this file (debug).\n"
      "    GFHOME_CONF      : homing config file (default /data/forgefirm.conf).\n"
      "\n"
      "  ^F (or SIGINT/SIGTERM) shuts down cleanly once motion is done.\n"
      "\n",
      progname);
}

static void print_version (void)
{
    printf("grblHAL-glowforge\n"
      "  Build type : %s\n"
      "  Compiler   : %s\n"
      "  Target     : %s\n"
      "  C flags    : %s\n"
      "  Built      : %s %s\n",
      DRV_BUILD_TYPE,
      DRV_COMPILER,
      DRV_TARGET,
      DRV_C_FLAGS[0] ? DRV_C_FLAGS : "(none)",
      __DATE__, __TIME__);
}

static void sig_handler (int signum)
{
    (void)signum;
    driver_request_exit();
    // A second signal falls through to the default action (hard kill; the
    // kernel dead man's switch e-stops any running job).
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
}

int main (int argc, char *argv[])
{
    int port = 0;
    int listen_fd = -1;

    progname = argv[0];
    set_eeprom_name("EEPROM.DAT");

    while(argc > 1) {
        argv++; argc--;
        if(argv[0][0] == '-') {

            if(!strcmp(argv[0], "--version")) {
                print_version();
                return EXIT_SUCCESS;
            }

            switch(argv[0][1]) {

                case 'e':
                    if(argc < 2 || !set_eeprom_name(argv[1])) {
                        printf("Option -e needs a settings file path (under 128 characters).\n");
                        print_usage(NULL);
                        return EXIT_FAILURE;
                    }
                    argv++; argc--;
                    break;

                case 'p':
                    if(argc < 2 || (port = atoi(argv[1])) <= 0 || port > 65535) {
                        printf("Option -p needs a TCP port (1-65535).\n");
                        print_usage(NULL);
                        return EXIT_FAILURE;
                    }
                    argv++; argc--;
                    break;

                case 'v':
                    print_version();
                    return EXIT_SUCCESS;

                case 'h':
                    print_usage(NULL);
                    return EXIT_SUCCESS;

                default:
                    print_usage(*argv);
                    return EXIT_FAILURE;
            }
        } else {
            print_usage(*argv);
            return EXIT_FAILURE;
        }
    }

    // Lock text and data so the SCHED_FIFO shipper never takes a major
    // page fault mid-stream. Root only: CAP_IPC_LOCK exempts the memlock
    // limit there, while under a finite RLIMIT_MEMLOCK a successful
    // MCL_FUTURE makes every later thread-stack mmap count against the
    // limit and thread creation fails outright - strictly worse than
    // running unlocked.
    if(geteuid() == 0 && mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        fprintf(stderr, "mlockall unavailable: %s (running unlocked)\n",
                strerror(errno));

    platform_init();

    if(port) {

        struct sockaddr_in server_addr = {0};

        // Close-on-exec: the homing runner is fork+exec'd from this
        // process and must not inherit the listen socket (a straggling
        // child would keep the port bound across a controller respawn).
        if((listen_fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)) < 0) {
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        int reuse = 1;
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);

        if(bind(listen_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            printf("Fatal: Unable to bind socket.\n");
            exit(-5);
        }

        listen(listen_fd, 1);

        // Non-blocking so serial_poll()'s accept() never stalls the
        // protocol thread.
        int flags = fcntl(listen_fd, F_GETFL, 0);
        if(flags != -1)
            fcntl(listen_fd, F_SETFL, flags | O_NONBLOCK);
    }

    serial_set_listen_fd(listen_fd);

    // Do not leave the EEPROM file inconsistent on exit; the stepper
    // stream registers its own atexit shutdown (kernel halt + latch
    // relock).
    atexit(eeprom_close);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);   // client disconnects surface as write errors

    // The protocol loop runs here and never returns; exit happens via the
    // realtime hook (exit request + motion done) or a signal.
    grbl_enter();

    return EXIT_SUCCESS;
}
