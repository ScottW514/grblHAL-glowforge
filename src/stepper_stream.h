/*
  stepper_stream.h - pulse-stream stepper engine (see stepper_stream.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Init (reads GFSINK/GFSINK_RATE/GFSINK_DEPTH_MS, opens the pulse device
// when GFSINK is set, applies the analog machine config, spawns the
// producer and shipper threads). Called from driver_init().
void gf_stream_init (void);

// Virtual step-clock frequency for hal.f_step_timer: 1000 x machine tick.
uint32_t gf_stream_vclk (void);
// Machine tick (pulse byte rate) in Hz.
uint32_t gf_stream_rate (void);

// hal.stepper entry points (call with the core lock held; the driver's
// handlers in driver.c take it).
void gf_stream_wakeup (void);
void gf_stream_go_idle (void);
void gf_stream_cycles_per_tick (uint32_t cycles);
void gf_stream_pulse (uint8_t step_bits, uint8_t dir_bits); // grblHAL bit order

// hal.driver_reset hook body: abort the stream (drop unshipped backlog,
// kernel controlled stop). Call only when sys.reset_pending.
void gf_stream_reset (void);

// Nonzero when the shipper hit an unrecoverable stream fault (underrun /
// write failure). Clears the flag; caller raises the alarm.
bool gf_stream_fault_take (void);

// Hand the pulse device to another process (the gfcloud homing runner)
// and take it back. Suspend succeeds only from a fully idle stream AND
// kernel (closing the flock'd fd mid-program is an emergency stop) -
// callers retry until it returns true. Resume reopens the device and
// re-applies the analog config, step_freq and stream state; false =
// device lost (raise an alarm). Both are no-ops in null-sink mode.
bool gf_stream_suspend (void);
bool gf_stream_resume (void);

void gf_stream_shutdown (void);
