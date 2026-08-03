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

// Homing: accelerometer bump-detect (glowforge_homing.c) - no physical
// switches exist. Home corner is the BACK-LEFT of the machine: X min
// (left) and Y min (+Y physically moves the gantry toward the FRONT,
// operator-verified). The origin is forced to the homed position, so
// the workspace is all-positive from that corner. X and Y home in
// separate cycles (one bump each); Z is excluded ($H never moves Z -
// it homes against the hall sensor in a later milestone). Standard
// fast-seek / slow-latch: the seek rail strike is harsher but the jolt
// only gets easier to detect, and accuracy comes from the gentle latch
// re-reference. At the seek rate the detector arms after ~12 mm, inside
// the pull-off runway only via the grinding-baseline guard: an approach
// that starts at/near the rail reads a grinding "baseline" and triggers
// at arm time.
#define DEFAULT_HOMING_ENABLE On
#define DEFAULT_HOMING_DIR_MASK (X_AXIS_BIT|Y_AXIS_BIT) // both home to min
#define DEFAULT_HOMING_CYCLE_0 X_AXIS_BIT
#define DEFAULT_HOMING_CYCLE_1 Y_AXIS_BIT
#define DEFAULT_HOMING_SEEK_RATE 1500.0f        // mm/min (25 mm/s seek)
// The locate pass runs at seek speed too: it is a second fast strike,
// not a precision re-find. Slow approaches CANNOT be detected on this
// machine - belt compliance turns slow-speed skipping into near-silent
// grinding (bench-measured under every threshold) - and approach speed
// does not affect accuracy because the rail itself is the reference.
#define DEFAULT_HOMING_FEED_RATE 1500.0f        // mm/min
// Pull-off must exceed the detector's arming distance AT SEEK RATE
// (~0.5 s = 12.5 mm at 25 mm/s), so a machine parked at the home
// pull-off re-homes with the detector armed before contact.
#define DEFAULT_HOMING_PULLOFF 15.0f            // mm
#define DEFAULT_HOMING_DEBOUNCE_DELAY 250       // ms
#define DEFAULT_HOMING_SINGLE_AXIS_COMMANDS On  // $HX / $HY for the bench
#define DEFAULT_HOMING_FORCE_SET_ORIGIN On      // homed corner = machine 0
