/*
  glowforge_homing.c - homing method dispatch for the Glowforge board

  Part of grblHAL-glowforge. The machine has no limit or home switches;
  homing method is selected at runtime by the operator (forgectrl web
  UI) through the shared config file /data/forgefirm.conf:

    homing_mode = gfcloud   $H runs the Glowforge web-service homing
                            sequence via the external one-shot runner
                            (gfhome.py: cloud vision homes X/Y to the
                            factory home corner, Z to the hall sensor).
    homing_mode = switches  $H falls through to the core homing cycle
                            (physical limit switches, once installed).
    homing_mode = none      $H falls through to the core, which rejects
                            it while homing is disabled ($22=0).

  The registered "H" system command shadows the core's (the command
  chain searches driver registrations first) and re-reads the config on
  every invocation, so mode changes apply without a restart.

  The gfcloud path hands /dev/glowforge to the runner for the whole
  session: the stream engine is suspended (only possible from a fully
  idle kernel - see gf_stream_suspend), the runner is spawned through
  /bin/sh, and the protocol loop keeps pumping real-time traffic so
  senders get status reports for the minutes the session can take. On
  success the machine position is set to the configured post-homing
  coordinates: the factory home corner is machine origin (back-left,
  workspace all-positive), the lens rests at top-of-travel. A soft
  reset (^X) aborts the session (SIGTERM, then SIGKILL); failures raise
  the homing-fail alarm.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

/* grbl headers first: glibc's stat.h (via fcntl.h) defines an st_mtime
 * macro that would otherwise mangle the field of that name in vfs.h */
#include "fflog.h"
#include "driver.h"
#include "glowforge_homing.h"
#include "glowforge_io.h"
#include "stepper_stream.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define CMD_DEFAULT       "/usr/sbin/gfhome.py"
#define TIMEOUT_S_DEFAULT 300.0f
#define SUSPEND_WAIT_MS   3000     /* decel tail + hold-current drop */
#define KILL_GRACE_MS     5000

/* Position anchor for external status readers (forgectrl): written on
 * homing success, right after the kernel position counters are zeroed,
 * so anchor + counters = machine position from then on. Removed
 * whenever the reference stops being trustworthy: controller start, a
 * homing session starting (the runner moves the machine and clears the
 * counters), or a stream fault. */
#define HOMED_ANCHOR      "/run/grblhal.homed"

/* The "key = value" config parser lives in glowforge_io (shared with
 * the cooling tunables); these are kept as local names only. */
#define cfg_read       gfio_conf_read
#define cfg_read_float gfio_conf_read_float

void gfhome_invalidate (void)
{
    unlink(HOMED_ANCHOR);
}

static void anchor_write (const float *home)
{
    FILE *f = fopen(HOMED_ANCHOR, "w");
    if(f) {
        fprintf(f, "%.3f %.3f %.3f\n",
                home[X_AXIS], home[Y_AXIS], home[Z_AXIS]);
        fclose(f);
    }
}

/* --- session orchestration -------------------------------------------- */

/* Pump the protocol while blocked in the session: flush/collect serial
 * traffic and run the real-time executive (status reports, overrides,
 * reset). Returns false once the system is aborting. */
static bool pump (long timeout_us)
{
    serial_wait(timeout_us);
    serial_poll();
    return protocol_execute_realtime();
}

static status_code_t gfcloud_home (sys_state_t entry_state)
{
    char cmd[256];
    if(cfg_read("gfcloud_home_cmd", cmd, sizeof(cmd)) != 0 || cmd[0] == '\0')
        strcpy(cmd, CMD_DEFAULT);

    uint32_t timeout_ms =
        (uint32_t)(cfg_read_float("gfcloud_home_timeout_s", TIMEOUT_S_DEFAULT) * 1000.0f);

    state_set(STATE_HOMING);
    gfhome_invalidate();   /* the session moves the machine */

    /* Hand the pulse device over; a just-finished move may still be
     * playing out its decel tail in the kernel. */
    bool suspended = false;
    uint32_t give_up = hal.get_elapsed_ticks() + SUSPEND_WAIT_MS;
    while(!(suspended = gf_stream_suspend()) &&
           (int32_t)(give_up - hal.get_elapsed_ticks()) > 0) {
        if(!pump(20000))
            break;
    }
    if(!suspended) {
        state_set(entry_state);
        return sys.abort ? Status_OK : Status_IdleError;
    }

    fflog(LOG_INFO, "gfhome: starting homing session: %s", cmd);

    /* Hand the runner its share of the budget so it gives up before the
     * SIGTERM deadline. Set pre-fork (env calls are not async-signal-
     * safe in the child); every getenv user runs on this thread. */
    char budget[16];
    snprintf(budget, sizeof(budget), "%u",
             timeout_ms > 60000 ? timeout_ms / 1000 - 30 : 30);
    setenv("GFHOME_TIMEOUT_S", budget, 1);

    pid_t pid = fork();
    if(pid == 0) {
        setpgid(0, 0);   /* own process group: SIGTERM reaches the whole tree */
        execl("/bin/sh", "sh", "-c", cmd, (char *)NULL);
        _exit(127);
    }
    unsetenv("GFHOME_TIMEOUT_S");
    if(pid < 0) {
        fflog(LOG_ERR, "gfhome: cannot spawn the homing runner");
        if(!gf_stream_resume())
            system_set_exec_alarm(Alarm_HomingFail);
        state_set(entry_state);
        return Status_IdleError;
    }

    int wstatus = 0;
    bool exited = false, aborted = false, term_sent = false, kill_sent = false;
    uint32_t deadline = hal.get_elapsed_ticks() + timeout_ms;
    uint32_t kill_at = 0;

    while(!exited) {
        pid_t r = waitpid(pid, &wstatus, WNOHANG);
        if(r == pid)
            exited = true;
        else if(r < 0)
            break;
        else {
            if(!pump(50000) && !aborted) {
                aborted = true;
                fflog(LOG_WARNING, "gfhome: abort - terminating the homing session");
            }
            bool overdue = (int32_t)(hal.get_elapsed_ticks() - deadline) > 0;
            if((aborted || overdue) && !term_sent) {
                if(overdue)
                    fflog(LOG_WARNING, "gfhome: homing session timed out");
                kill(-pid, SIGTERM);
                term_sent = true;
                kill_at = hal.get_elapsed_ticks() + KILL_GRACE_MS;
            } else if(term_sent && !kill_sent &&
                       (int32_t)(hal.get_elapsed_ticks() - kill_at) > 0) {
                kill(-pid, SIGKILL);
                kill_sent = true;
            }
        }
    }

    bool homed = exited && !aborted && !term_sent &&
                  WIFEXITED(wstatus) && WEXITSTATUS(wstatus) == 0;

    bool resumed = gf_stream_resume();

    if(!resumed || !homed) {
        if(exited && !aborted && !term_sent && !homed)
            fflog(LOG_ERR, "gfhome: homing runner failed (status %d)",
                  WIFEXITED(wstatus) ? WEXITSTATUS(wstatus) : -1);
        /* Same shape as a failed core homing cycle: back to idle with
         * the alarm queued - the protocol loop broadcasts ALARM and
         * enters the alarm state. On a clean abort the reset path owns
         * both; a lost device alarms unconditionally. */
        if(!resumed || !aborted) {
            system_set_exec_alarm(Alarm_HomingFail);
            if(state_get() == STATE_HOMING)
                state_set(STATE_IDLE);
        }
        return Status_OK;
    }

    /* Homed: the head sits at the factory home position. Machine origin
     * is that corner (back-left, +Y toward the front), workspace all-
     * positive; the lens rests at the top-of-travel hall reference.
     * NOTE: settings.axis[].max_travel is stored negative. */
    float home[N_AXIS];
    home[X_AXIS] = cfg_read_float("gfcloud_home_x", 0.0f);
    home[Y_AXIS] = cfg_read_float("gfcloud_home_y", 0.0f);
    home[Z_AXIS] = cfg_read_float("gfcloud_home_z",
                                   -settings.axis[Z_AXIS].max_travel);

    uint_fast8_t idx;
    for(idx = 0; idx < N_AXIS; idx++) {
        sys.position[idx] = lroundf(home[idx] * settings.axis[idx].steps_per_mm);
        sys.home_position[idx] = home[idx];
        sys.work_envelope.min.values[idx] = 0.0f;
        sys.work_envelope.max.values[idx] = -settings.axis[idx].max_travel;
    }
    sys.homed.mask = X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT;
    sync_position();

    gf_stream_clear_position();
    anchor_write(home);

    if(grbl.on_homing_completed)
        grbl.on_homing_completed(
            (axes_signals_t){ .mask = X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT }, true);
    report_add_realtime(Report_Homed);

    state_set(STATE_IDLE);
    st_go_idle();
    grbl.report.feedback_message(Message_None);

    fflog(LOG_NOTICE, "gfhome: homed - X%.2f Y%.2f Z%.2f",
          home[X_AXIS], home[Y_AXIS], home[Z_AXIS]);

    return Status_OK;
}

static status_code_t home_cmd (sys_state_t state, char *args)
{
    (void)args;

    char mode[24] = "";
    cfg_read("homing_mode", mode, sizeof(mode));
    if(strcmp(mode, "gfcloud"))
        return Status_Unhandled;   /* switches/none: core $H semantics */

    if(!(state == STATE_IDLE || state == STATE_ALARM))
        return Status_IdleError;

    return gfcloud_home(state);
}

static const sys_command_t homing_command_list[] = {
    { "H", home_cmd }
};

static sys_commands_t homing_commands = {
    .n_commands = sizeof(homing_command_list) / sizeof(sys_command_t),
    .commands = homing_command_list
};

void gfhome_init (void)
{
    gfhome_invalidate();   /* a fresh controller is not homed */
    system_register_commands(&homing_commands);
}
