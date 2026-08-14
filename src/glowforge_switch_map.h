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
#define SW_BIT_ESTOP      4
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
   - e-stop (bit 4) is a sense line resting ACTIVE on a healthy
     machine; when gating is enabled, the assertion is the line
     DROPPING. */
static inline control_signals_t gfsw_map_bits (const uint8_t *sw,
                                               bool estop_gates)
{
    control_signals_t signals = {0};

    signals.safety_door_ajar =
        !gfsw_bit_set(sw, SW_BIT_DOORS) || gfsw_bit_set(sw, SW_BIT_INTERLOCK);
    if(estop_gates)
        signals.e_stop = !gfsw_bit_set(sw, SW_BIT_ESTOP);

    return signals;
}
