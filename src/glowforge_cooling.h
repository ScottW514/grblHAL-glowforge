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
