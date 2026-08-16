/*
  glowforge_laser.h - laser spindle + operator arming (see glowforge_laser.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>

// Register the laser spindle (M3/M4 + S -> pulse-stream power bytes and
// the per-tick FIRE bit). Called from driver_init().
void gflaser_init (void);

// Periodic work on the protocol thread (realtime hook): close the armed
// window (idle grace / alarm / homing -> relock the kernel laser latch)
// and surface deferred warnings from the producer thread.
void gflaser_poll (void);

// Immediate disarm + latch relock (driver reset / stream fault paths).
void gflaser_disarm (void);

// True while the arm flow is blocked waiting for the operator's button
// press: that press is the arm's, and the button toggle must not act on
// it (glowforge_switches.c).
bool gflaser_arming (void);
