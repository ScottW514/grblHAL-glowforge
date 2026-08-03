/*
  glowforge_homing.h - accelerometer bump-detect homing signals

  Part of grblHAL-glowforge. The factory machine has no X/Y home
  switches; the head's LIS2HH12 accelerometer senses the rail-contact
  jolt instead and is surfaced to the core as a virtual limit switch.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>

#include "grbl/hal.h"

/* Spawn the (idle) monitor thread. */
void gfhome_init (void);

/* hal.limits.enable: a non-zero homing_cycle arms the bump detector for
 * those axes (the whole seek/pull-off/locate sequence of one cycle runs
 * under a single arm); zero disarms it. */
void gfhome_enable (bool on, axes_signals_t homing_cycle);

/* hal.limits.get_state: triggered cycle axes appear in .min while the
 * short post-contact hold lasts. Zero outside homing (the machine has no
 * physical limit inputs). */
limit_signals_t gfhome_get_state (void);
