/*
  driver.h - grblHAL-glowforge driver interface

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

// The "IRQ" lock: a recursive mutex standing in for interrupt masking.
// hal.irq_disable/irq_enable map to it, the atomics helpers take it, and
// the stepper producer thread holds it around every core stepper
// interrupt callback. Lock order: core lock -> gf stream lock.
void gf_core_lock_init (void);
void gf_core_lock (void);
void gf_core_unlock (void);

// Request a clean shutdown (^F over a stream, SIGINT/SIGTERM). The
// realtime hook exits once motion is done. Async-signal-safe.
void driver_request_exit (void);
