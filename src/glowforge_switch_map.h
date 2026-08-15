/*
  glowforge_switch_map.h - the pure EV_SW bit -> control-signal mapping

  Part of grblHAL-glowforge. Split from glowforge_switches.c so the
  host unit test (tests/switch_map_test.c) can exercise the exact
  decode the controller gates motion on - a polarity flip here would
  make a Pro's remote interlock read satisfied-while-open.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "grbl/hal.h"

#include <stdbool.h>
#include <stdint.h>

/* EV_SW bits per the device tree (see glowforge_switches.c for the
   semantics and the SERVICES.md switch map for the shared contract). */
#define SW_BIT_DOORS      3
#define SW_BIT_HV_ENABLE  4   /* readback of the HV_ENABLE output; monitoring only */
#define SW_BIT_INTERLOCK  5

#define SW_BYTES          2

static inline bool gfsw_bit_set (const uint8_t *sw, unsigned bit)
{
    return !!(sw[bit / 8] & (1u << (bit % 8)));
}

/* Maps a raw EVIOCGSW bitmask onto the core's control signals:
   - doors (bit 3) is the series lid chain, active = closed: INACTIVE
     means the lid is open -> safety door ajar.
   - interlock (bit 5) is the remote-interlock loop with INVERTED
     sense, active = loop OPEN (lockout engaged) -> safety door ajar.
     Basic/Plus ship the connector jumpered, so the bit rests inactive.
   - hv_enable (bit 4) is the readback of the HV_ENABLE output and
     gates nothing: it is low at idle and high only while a run feeds
     the charge-pump watchdog with the lid closed. */
static inline control_signals_t gfsw_map_bits (const uint8_t *sw)
{
    control_signals_t signals = {0};

    signals.safety_door_ajar =
        !gfsw_bit_set(sw, SW_BIT_DOORS) || gfsw_bit_set(sw, SW_BIT_INTERLOCK);

    return signals;
}

/* The control signals the core should see, given its state. This is the
   single policy behind both hal.control.get_state() and the edge
   delivery: the safety-door signal is withheld while the core is IDLE,
   JOG or HOMING and shown as-is in every other state. Rationale: the beam
   is blocked in hardware whenever the lid or the interlock loop is open;
   the lid is opened at idle every time material is loaded; and a door the
   core sees while idle - at boot, on a stop, or as an edge - strands the
   controller in Door until a cycle start. In a running or held job, a
   tool change, or an existing door state the signal is live, so a job
   started with the lid open parks on the first poll and a mid-job open
   parks exactly as before. */
static inline control_signals_t gfsw_visible (control_signals_t now,
                                              sys_state_t state)
{
    if(state == STATE_IDLE || state == STATE_JOG || state == STATE_HOMING)
        now.safety_door_ajar = Off;

    return now;
}

/* Diffs what the core should see now against what it has already been
   told: *on = newly asserted signals, *off = signals to report as
   deasserted (deasserted flag set). Returns the new "told" state. */
static inline control_signals_t gfsw_edges (control_signals_t want,
                                            control_signals_t delivered,
                                            control_signals_t *on,
                                            control_signals_t *off)
{
    on->bits = want.bits & ~delivered.bits;
    off->bits = delivered.bits & ~want.bits;
    if(off->bits)
        off->deasserted = On;

    return want;
}
