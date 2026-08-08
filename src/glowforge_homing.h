/*
  glowforge_homing.h - homing method dispatch (see glowforge_homing.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

// Register the "$H" system command (shadows the core's; dispatches on
// the homing_mode key in /data/forgefirm.conf, GFHOME_CONF overrides
// the path). Called from driver_init().
void gfhome_init (void);

// Drop the /run position anchor: the homed reference is no longer
// trustworthy (stream fault, position lost). Homing success rewrites it.
void gfhome_invalidate (void);
