/*
  glowforge_io.h - Glowforge kernel-driver sysfs / pulse-device access

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>
#include <stddef.h>

// Attribute paths are relative to /sys/glowforge/ (e.g. "cnc/state",
// "pic/x_step_current"). All return 0 on success, -1 on failure.
void gfio_set_hw (bool active);
// 0 on a complete write; GFIO_ENOATTR when the attribute does not exist
// (module not loaded / older kernel); GFIO_EREJECT when the store refused
// or truncated the value.
#define GFIO_ENOATTR (-2)
#define GFIO_EREJECT (-1)
int gfio_wr_attr (const char *attr, const char *val);
int gfio_rd_attr (const char *attr, char *buf, size_t len);

// Open the pulse device and take the exclusive flock (the kernel dead
// man's switch). Returns the fd, or -1. The _nb variant fails instead
// of blocking when another process still holds the lock. Under the
// forgectrl device broker (GF_PULSE_FD in the environment) both return
// the inherited fd instead of opening - see glowforge_io.c.
int gfio_open_pulse_dev (const char *path);
int gfio_open_pulse_dev_nb (const char *path);

// True when the pulse device came in from the broker: the device stays
// open across handovers (never close it; the rail never cycled, so no
// settle is needed on takeover).
bool gfio_pulse_inherited (void);

// Factory analog machine config (print-header ground truth): x8 XY
// microstepping, slow-decay mode, Z locked in the pulse stream, laser
// latched out, PIC hold currents. The kernel does not restore any of
// this after a module reload.
void gfio_analog_config (void);

// Factory run/idle current scheme: full torque only while motion plays.
void gfio_currents_run (void);
void gfio_currents_hold (void);

// The shared machine config /data/forgefirm.conf ("key = value" lines,
// '#' comments; GFHOME_CONF overrides the path). Written by the
// forgectrl web UI; consumers re-read at their natural boundaries
// (homing per $H, cooling per flood start). Returns 0 when the key
// exists; the float variant falls back on a missing or unparsable key.
int gfio_conf_read (const char *key, char *val, size_t len);
float gfio_conf_read_float (const char *key, float fallback);
