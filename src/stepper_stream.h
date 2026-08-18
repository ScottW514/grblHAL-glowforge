/*
  stepper_stream.h - pulse-stream stepper engine (see stepper_stream.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Hardware PWM resolution: the power byte carries 7 bits, written raw
// into PWMSAR against this period, so 127 is full duty.
#define GF_PWM_PERIOD 127

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

// Laser power/fire transition at the current virtual time (call with the
// core lock held). power is the raw 7-bit PWM duty (127 = full); fire
// drives the per-tick FIRE bit from this stream position onward. Dropped
// while the stream is idle: every laser block re-asserts power at its
// first segment, and the end-of-data backstop keeps the lines low.
void gf_stream_laser (uint8_t power, bool fire);

// Duty update with no change of fire state, for the synchronous spindle
// path: an S word executed with no motion in progress reaches the driver
// through set_state, not through the per-segment updates, and dropping
// its speed would leave the next move cutting at a stale duty.
void gf_stream_laser_power (uint8_t power);

// Laser dose model. The per-segment value the core computes is rendered
// by the shipper either as an analog duty (a power byte, period 0 here)
// or as FIRE-bit density at full duty: a fixed base period of
// period_ticks whose on-count is dithered between adjacent integers,
// the remainder carried so densities finer than one tick per period
// still average out. Density is what the tube's dead band below its
// lasing threshold requires - every pulse it emits is full-power, so no
// commanded level lands in the band. Selected per arm.
void gf_stream_laser_model (uint32_t period_ticks);

// Laser arming state (glowforge_laser.c owns the policy). While armed an
// underrun faults the stream instead of the stop/run retry: a restarted
// run resets the hardware PWM duty, so replaying queued fire bits would
// fire at ~full power.
void gf_stream_laser_arm (bool armed);

// Serialized cnc/laser_latch write (lock = 1 locks the latch). Every
// latch write in the process goes through here so the shipper's
// run-start relight decision is atomic against a concurrent disarm.
void gf_stream_laser_latch (bool lock);

// hal.driver_reset hook body: abort the stream (drop unshipped backlog,
// kernel controlled stop). Call only when sys.reset_pending.
void gf_stream_reset (void);

// Nonzero when the shipper hit an unrecoverable stream fault (underrun /
// write failure). Clears the flag; caller raises the alarm.
bool gf_stream_fault_take (void);

// Zero the kernel position counters (homing established a reference;
// external status readers add the home offset to the counters).
void gf_stream_clear_position (void);

// Hand the pulse device to another process (the gfcloud homing runner)
// and take it back. Suspend succeeds only from a fully idle stream AND
// kernel (closing the flock'd fd mid-program is an emergency stop) -
// callers retry until it returns true. Resume reopens the device and
// re-applies the analog config, step_freq and stream state; false =
// device lost (raise an alarm). Both are no-ops in null-sink mode.
bool gf_stream_suspend (void);
bool gf_stream_resume (void);

void gf_stream_shutdown (void);
