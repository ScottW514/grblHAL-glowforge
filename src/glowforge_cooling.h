/*
  glowforge_cooling.h - fan / coolant management (see glowforge_cooling.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "grbl/hal.h"

// Apply the base machine state: water pump on, TEC off, purge air on,
// fans at factory idle. Called from driver_init().
void gfcool_init (void);

// hal.coolant backends. Flood (M8) = the cut-profile fans at factory run
// values; flood off (M9) = cooldown profile + timer back to idle.
void gfcool_coolant_set (coolant_state_t state);
coolant_state_t gfcool_coolant_get (void);

// Periodic work on the protocol thread (realtime hook): cooldown expiry,
// 1 Hz water-temperature monitoring.
void gfcool_poll (void);

// Laser fire gate: false while a coolant flow FAULT stands or the
// coolant is over the run ceiling (with resume-gate hysteresis). Read
// from the stepper producer thread as well - plain bool reads, a
// segment of staleness is acceptable.
bool gfcool_fire_ok (void);

// Armed-window hook from glowforge_laser.c: while armed the run fan
// profile (and with it the flow interrogation) is forced on, whatever
// the sender's M8/M9 state, and a flow SUSPECT/FAULT verdict takes the
// safe posture (feed hold + forced cooling; fire is gated separately).
void gfcool_laser_armed (bool armed);
