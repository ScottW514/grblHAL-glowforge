/*
  glowforge_cooling.h - cooling-service client (see glowforge_cooling.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "grbl/hal.h"

// Client setup (report target, verdict cache). The cooling engine and
// the machine's thermal posture live in forgectrl; the contract is
// forgectrl docs/SERVICES.md. Called from driver_init().
void gfcool_init (void);

// hal.coolant backends. Flood (M8/M9) drives the job-state reports;
// the engine applies the fan profiles.
void gfcool_coolant_set (coolant_state_t state);
coolant_state_t gfcool_coolant_get (void);

// Periodic work on the protocol thread (realtime hook): the ~1 Hz
// level-triggered job-state report, the verdict-file read, and the
// hold / resume / fallback enforcement.
void gfcool_poll (void);

// Laser fire gate: false while the engine's verdict says so - flow
// FAULT or coolant over the run ceiling - and whenever the verdict is
// missing or stale (the engine being gone must look like a fault).
// Read from the stepper producer thread as well - cached flag reads,
// no file IO on that path.
bool gfcool_fire_ok (void);

// Armed-window hook from glowforge_laser.c: reported to the engine,
// which forces the run fan profile and flow interrogation while armed,
// whatever the sender's M8/M9 state.
void gfcool_laser_armed (bool armed);
