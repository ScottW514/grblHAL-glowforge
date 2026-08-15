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
    - hv_enable (bit 4) is the readback of the HV_ENABLE output and
      never gates: neither state asserts anything

  Also covers the visibility policy (gfsw_visible / gfsw_edges): the
  door signal is hidden from the core while IDLE, JOG or HOMING and
  delivered the moment it is in any other state.

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

static uint8_t *mask (bool doors, bool hv_enable, bool interlock)
{
    static uint8_t sw[SW_BYTES];
    sw[0] = (uint8_t)((doors ? 1u << SW_BIT_DOORS : 0)
                    | (hv_enable ? 1u << SW_BIT_HV_ENABLE : 0)
                    | (interlock ? 1u << SW_BIT_INTERLOCK : 0));
    sw[1] = 0;
    return sw;
}

int main (void)
{
    control_signals_t s;

    /* Healthy idle machine: lid closed, HV_ENABLE readback low,
       interlock loop closed (bit inactive). Nothing gates. */
    s = gfsw_map_bits(mask(true, false, false));
    expect("healthy machine gates nothing", s.bits != 0, false);

    /* Lid open -> safety door ajar. */
    s = gfsw_map_bits(mask(false, false, false));
    expect("open lid is a door event", s.safety_door_ajar, true);

    /* Remote interlock loop OPEN = bit ACTIVE (inverted sense) ->
       safety door ajar. THE polarity this test exists for. */
    s = gfsw_map_bits(mask(true, false, true));
    expect("open interlock loop is a door event", s.safety_door_ajar, true);

    /* Interlock loop closed (jumpered Basic/Plus) with the lid closed
       must NOT read as a door event. */
    s = gfsw_map_bits(mask(true, false, false));
    expect("closed interlock loop is not a door event",
           s.safety_door_ajar, false);

    /* Both open: still (only) a door event. */
    s = gfsw_map_bits(mask(false, false, true));
    expect("lid + interlock both open is a door event",
           s.safety_door_ajar, true);

    /* The HV_ENABLE readback (bit 4) is telemetry: high (a run in
       progress) or low (idle) asserts nothing, in particular not e_stop. */
    s = gfsw_map_bits(mask(true, true, false));
    expect("hv_enable high gates nothing", s.bits != 0, false);
    s = gfsw_map_bits(mask(true, false, false));
    expect("hv_enable low gates nothing", s.bits != 0, false);
    s = gfsw_map_bits(mask(false, true, false));
    expect("open lid with hv_enable high is only a door event",
           s.safety_door_ajar && !s.e_stop, true);

    /* ---- visibility policy: what the core sees, by its state ---- */

    control_signals_t open_lid = gfsw_map_bits(mask(false, false, false));
    control_signals_t open_loop = gfsw_map_bits(mask(true, false, true));

    /* Idle, jog and homing: an open lid or loop is hidden (the beam is
       blocked in hardware; a door seen here strands the core in Door). */
    expect("open lid hidden while IDLE", gfsw_visible(open_lid, STATE_IDLE).safety_door_ajar, false);
    expect("open lid hidden while JOG", gfsw_visible(open_lid, STATE_JOG).safety_door_ajar, false);
    expect("open lid hidden while HOMING", gfsw_visible(open_lid, STATE_HOMING).safety_door_ajar, false);
    expect("open loop hidden while IDLE", gfsw_visible(open_loop, STATE_IDLE).safety_door_ajar, false);

    /* Any job-time state: visible as-is. */
    expect("open lid visible in CYCLE", gfsw_visible(open_lid, STATE_CYCLE).safety_door_ajar, true);
    expect("open lid visible in HOLD", gfsw_visible(open_lid, STATE_HOLD).safety_door_ajar, true);
    expect("open lid visible in SAFETY_DOOR", gfsw_visible(open_lid, STATE_SAFETY_DOOR).safety_door_ajar, true);
    expect("open lid visible in TOOL_CHANGE", gfsw_visible(open_lid, STATE_TOOL_CHANGE).safety_door_ajar, true);
    expect("open loop visible in CYCLE", gfsw_visible(open_loop, STATE_CYCLE).safety_door_ajar, true);

    /* ---- delivery sequence: the load-material lid cycle, then a job ---- */

    control_signals_t told = {0}, on, off, want;
    control_signals_t closed = gfsw_map_bits(mask(true, false, false));

    /* Lid opened and closed at idle: nothing is delivered either way. */
    want = gfsw_visible(open_lid, STATE_IDLE);
    told = gfsw_edges(want, told, &on, &off);
    expect("idle lid open delivers nothing", on.bits == 0 && off.bits == 0, true);
    want = gfsw_visible(closed, STATE_IDLE);
    told = gfsw_edges(want, told, &on, &off);
    expect("idle lid close delivers nothing", on.bits == 0 && off.bits == 0, true);

    /* Job started with the lid open: delivered on the first non-idle poll,
       and only once. */
    want = gfsw_visible(open_lid, STATE_IDLE);
    told = gfsw_edges(want, told, &on, &off);
    want = gfsw_visible(open_lid, STATE_CYCLE);
    told = gfsw_edges(want, told, &on, &off);
    expect("lid open at job start is delivered", on.safety_door_ajar && !on.deasserted, true);
    told = gfsw_edges(gfsw_visible(open_lid, STATE_SAFETY_DOOR), told, &on, &off);
    expect("...and only once", on.bits == 0 && off.bits == 0, true);

    /* Lid closed while parked: reported as a deassertion, so the next open
       is delivered again. */
    want = gfsw_visible(closed, STATE_SAFETY_DOOR);
    told = gfsw_edges(want, told, &on, &off);
    expect("lid close while parked reports deassert", off.safety_door_ajar && off.deasserted, true);
    want = gfsw_visible(open_lid, STATE_CYCLE);
    told = gfsw_edges(want, told, &on, &off);
    expect("mid-job open is delivered", on.safety_door_ajar, true);

    /* Reset to idle with the lid still open: the core is told it cleared
       (it ignores door deasserts) and a later idle close stays silent. */
    want = gfsw_visible(open_lid, STATE_IDLE);
    told = gfsw_edges(want, told, &on, &off);
    expect("back to idle clears the delivered door", off.safety_door_ajar && told.safety_door_ajar == Off, true);
    want = gfsw_visible(closed, STATE_IDLE);
    told = gfsw_edges(want, told, &on, &off);
    expect("idle close after that stays silent", on.bits == 0 && off.bits == 0, true);

    if(failures) {
        printf("FAIL: %d switch-map case(s)\n", failures);
        return EXIT_FAILURE;
    }
    printf("PASS: switch-map decode truth table holds\n");
    return EXIT_SUCCESS;
}
