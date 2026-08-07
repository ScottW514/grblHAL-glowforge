/*
  boards/glowforge.h - Glowforge factory-board machine constants

  Part of grblHAL-glowforge, the ForgeFIRM grblHAL driver for the stock
  Glowforge (Basic/Plus/Pro) i.MX6 control board.

  Every value below is measured/derived from the factory machine, not
  guessed. Sources: kernel-module-glowforge/UAPI.md (feeder contract),
  forgefirm/docs/BRINGUP.md (hardware facts bank), and the factory pulse
  streams analyzed with forgefirm/scripts/bench/puls_profile.py:
  - XY: 0.15 mm per full step at x8 microstepping -> 53.333 usteps/mm.
  - Z: 0.3534 mm per half-step -> 2.832 half-steps/mm, ~10.6 mm travel.
  - Travel moves peak 202 mm/s vector with ~700 mm/s2 ramps on v2.6.0
    factory firmware (header HAxr=132/HAyr=112/HAar=133 at ~5.3 mm/s2 per
    unit; the 2018 firmware ramped at ~1000, so 700/590 is conservative).
  - Z cadence <= ~16 half-steps/s observed -> 300 mm/min cap.

  This file is force-included (via my_machine.h) into every translation
  unit including the grblHAL core, so the core's #ifndef-guarded defaults
  in config.h pick these values up. Keep it to preprocessor defines only.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#define BOARD_GLOWFORGE

#define DEFAULT_X_STEPS_PER_MM 53.333f
#define DEFAULT_Y_STEPS_PER_MM 53.333f
#define DEFAULT_Z_STEPS_PER_MM 2.832f

#define DEFAULT_X_MAX_RATE 12000.0f // mm/min (200 mm/s; factory travels at 197-202)
#define DEFAULT_Y_MAX_RATE 12000.0f
#define DEFAULT_Z_MAX_RATE 300.0f

#define DEFAULT_X_ACCELERATION 700.0f // mm/s^2
#define DEFAULT_Y_ACCELERATION 590.0f
#define DEFAULT_Z_ACCELERATION 50.0f

#define DEFAULT_X_MAX_TRAVEL 495.0f // mm
#define DEFAULT_Y_MAX_TRAVEL 279.0f
#define DEFAULT_Z_MAX_TRAVEL 10.6f

// It is a laser: $32 on by default so senders' M3/M4 dynamic-power
// semantics work without a settings dance. Fire remains impossible at
// this stage (locked spindle + hardware laser latch + no laser bit in
// the pulse stream).
#define DEFAULT_LASER_MODE On

// Homing is disabled: the machine has no limit or home switches yet.
// Limit-switch homing is the planned path; until the switches exist,
// $H is rejected and the limit signals are stubbed (driver.c). When
// homing returns: home corner is the BACK-LEFT of the machine - X min
// (left) and Y min (+Y physically moves the gantry toward the FRONT,
// operator-verified) - with the origin forced to the homed position so
// the workspace is all-positive from that corner; Z homes against the
// hall sensor only (never blind-drive Z).
#define DEFAULT_HOMING_ENABLE Off
