/*
  switch_map_test.c - truth table for the EV_SW -> control-signal decode

  Part of grblHAL-glowforge. Host unit test for the exact mapping the
  controller gates motion on (src/glowforge_switch_map.h). The
  polarity facts under test are the shared contract (forgectrl
  docs/SERVICES.md switch map):

    - doors (bit 3) is active = lid CLOSED (series chain)
    - interlock (bit 5) is INVERTED: active = loop OPEN (lockout);
      a flip here would read a Pro's remote interlock as satisfied
      while it is open
    - e-stop (bit 4) rests ACTIVE on a healthy machine; the assertion
      is the line dropping, and only when gating is enabled

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "glowforge_switch_map.h"

#include <stdio.h>
#include <stdlib.h>

static int failures = 0;

static void expect (const char *name, bool got, bool want)
{
    if(got != want) {
        printf("FAIL: %s: got %d, want %d\n", name, got, want);
        failures++;
    } else {
        printf("ok:   %s\n", name);
    }
}

static uint8_t *mask (bool doors, bool estop, bool interlock)
{
    static uint8_t sw[SW_BYTES];
    sw[0] = (uint8_t)((doors ? 1u << SW_BIT_DOORS : 0)
                    | (estop ? 1u << SW_BIT_ESTOP : 0)
                    | (interlock ? 1u << SW_BIT_INTERLOCK : 0));
    sw[1] = 0;
    return sw;
}

int main (void)
{
    control_signals_t s;

    /* Healthy machine: lid closed, e-stop sense resting active,
       interlock loop closed (bit inactive). Nothing gates. */
    s = gfsw_map_bits(mask(true, true, false), false);
    expect("healthy machine gates nothing", s.bits != 0, false);

    /* Lid open -> safety door ajar. */
    s = gfsw_map_bits(mask(false, true, false), false);
    expect("open lid is a door event", s.safety_door_ajar, true);

    /* Remote interlock loop OPEN = bit ACTIVE (inverted sense) ->
       safety door ajar. THE polarity this test exists for. */
    s = gfsw_map_bits(mask(true, true, true), false);
    expect("open interlock loop is a door event", s.safety_door_ajar, true);

    /* Interlock loop closed (jumpered Basic/Plus) with the lid closed
       must NOT read as a door event. */
    s = gfsw_map_bits(mask(true, true, false), false);
    expect("closed interlock loop is not a door event",
           s.safety_door_ajar, false);

    /* Both open: still (only) a door event. */
    s = gfsw_map_bits(mask(false, true, true), false);
    expect("lid + interlock both open is a door event",
           s.safety_door_ajar, true);

    /* E-stop gating disabled (the default): a dropped sense line does
       not assert e_stop (the factory board drops it during any motion). */
    s = gfsw_map_bits(mask(true, false, false), false);
    expect("dropped e-stop sense ignored when gating is off",
           s.e_stop, false);

    /* E-stop gating enabled: the assertion is the line DROPPING... */
    s = gfsw_map_bits(mask(true, false, false), true);
    expect("dropped e-stop sense asserts when gating is on", s.e_stop, true);

    /* ...and the resting-active line is not an assertion. */
    s = gfsw_map_bits(mask(true, true, false), true);
    expect("resting e-stop sense does not assert", s.e_stop, false);

    if(failures) {
        printf("FAIL: %d switch-map case(s)\n", failures);
        return EXIT_FAILURE;
    }
    printf("PASS: switch-map decode truth table holds\n");
    return EXIT_SUCCESS;
}
