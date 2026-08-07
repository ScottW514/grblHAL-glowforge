/*
  glowforge_io.h - Glowforge kernel-driver sysfs / pulse-device access

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stddef.h>

// Attribute paths are relative to /sys/glowforge/ (e.g. "cnc/state",
// "pic/x_step_current"). All return 0 on success, -1 on failure.
int gfio_wr_attr (const char *attr, const char *val);
int gfio_rd_attr (const char *attr, char *buf, size_t len);

// Open the pulse device and take the exclusive flock (the kernel dead
// man's switch). Returns the fd, or -1. The _nb variant fails instead
// of blocking when another process still holds the lock.
int gfio_open_pulse_dev (const char *path);
int gfio_open_pulse_dev_nb (const char *path);

// Factory analog machine config (print-header ground truth): x8 XY
// microstepping, slow-decay mode, Z locked in the pulse stream, laser
// latched out, PIC hold currents. The kernel does not restore any of
// this after a module reload.
void gfio_analog_config (void);

// Factory run/idle current scheme: full torque only while motion plays.
void gfio_currents_run (void);
void gfio_currents_hold (void);
