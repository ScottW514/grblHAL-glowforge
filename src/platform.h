/*
  platform.h - Linux platform interface (threads, clock, sleep, stdin)

  Part of grblHAL-glowforge. Descended from the grblHAL Simulator's
  platform layer (Copyright (c) 2014 Adam Shelly), reduced to the Linux
  implementation - this driver targets the Glowforge board's Linux
  userspace only.

  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <inttypes.h>
#include <pthread.h>

typedef struct {
    pthread_t tid;
    int exit;
} plat_thread_t;

typedef void *(*plat_threadfunc_t)(void *);
#define PLAT_THREAD_FUNC(name, arg) void *name (void *arg)

void platform_init (void);
void platform_terminate (void);

plat_thread_t *platform_start_thread (plat_threadfunc_t func);
void platform_stop_thread (plat_thread_t *thread);
void platform_kill_thread (plat_thread_t *thread);

uint32_t platform_ns (void);        // monotonically increasing ns since program start (rolls over)
void platform_sleep (long microsec);

uint8_t platform_poll_stdin (void); // non-blocking; 0 = no data, 0xFF = EOF
