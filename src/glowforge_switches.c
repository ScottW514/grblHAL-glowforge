/*
  glowforge_switches.c - safety switch inputs (lid, interlock)

  Part of grblHAL-glowforge

  Copyright (c) 2026 Scott Wiederhold

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
  The machine's switches arrive as EV_SW bits on a gpio-keys input device.
  State is read with EVIOCGSW, never a grab: forgectrl polls the same device
  for the status panel, and the laser arm flow reads the button from its own
  descriptor.

  Two of the bits gate motion, mapped onto the core's safety-door signal
  because that is what they mean - the machine cannot fire:

    doors     (bit 3) inactive  the lid is not closed. This is the series
                                combination the hardware safety chain itself
                                uses, not the individual door switches.
    interlock (bit 5) active    the remote-interlock loop is OPEN, i.e. the
                                regulatory lockout is engaged. The sense is
                                inverted relative to the door switches;
                                Basic/Plus ship the connector jumpered, so the
                                bit rests inactive there.

  Opening the lid mid-job therefore parks the job in the door state; once it
  is closed a cycle start resumes it. The hardware does the same to the beam
  regardless - the button latch sets the moment the lid opens, and the kernel
  sets the interlock latch when the loop opens. While the core is IDLE, JOG
  or HOMING the door signal is deliberately hidden from it (gfsw_visible,
  applied to both get_state() and the edge delivery): the lid is opened at
  idle every time material is loaded, the beam is blocked in hardware anyway,
  and a door seen while idle - at boot, on a stop, or as an edge - strands
  the controller in Door until a cycle start. The signal becomes visible,
  and is delivered, the moment the core is in any other state, so a job
  started with the lid open parks on the first poll.

  Bit 4 (hv_enable) is the readback of the board's HV_ENABLE output, not an
  input: it is low at idle and high only while a run feeds the charge-pump
  watchdog with the lid closed. It is telemetry (forgectrl shows it) and gates
  nothing here; the core's e_stop signal is not wired to anything on this
  hardware.

  Bit 6 (interlock latch tripped) is deliberately not gated on: its resting
  state on a healthy machine is not characterized, and a false assertion would
  wedge every job. The hardware chain enforces it regardless.

  The button (bit 2) is not a control signal: the laser arm flow reads it
  through gfsw_read_raw() as the operator's consent, and nothing else in
  the controller acts on it.

  Test hook: with GF_SWITCH_FILE set (and no device, i.e. a null-sink host
  build), the EV_SW word is read from that file instead - an integer,
  decimal or 0x-hex, holding the bitmask exactly as EVIOCGSW would return
  it. The host harnesses use it to open the lid, break the interlock loop
  and press the button against the real gating code.
*/

#include "fflog.h"
#include "glowforge_switches.h"
#include "glowforge_switch_map.h"

#include "grbl/state_machine.h"

#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SWITCH_DEV        "/dev/input/event0"

static int sw_fd = -1;
static const char *fake_path;              /* GF_SWITCH_FILE (host tests) */
static control_signals_t state = {0};      /* raw reading of the switch device */
static control_signals_t delivered = {0};  /* asserted signals the core has been told about */

bool gfsw_available (void)
{
    return sw_fd >= 0 || fake_path != NULL;
}

/* Reads the raw EV_SW word (EVIOCGSW layout, SW_BYTES bytes). Returns
   false when there is no switch source or it cannot be read right now;
   callers keep their previous state then, so a transient read failure
   never fakes an edge. */
bool gfsw_read_raw (uint8_t *sw)
{
    memset(sw, 0, SW_BYTES);

    if(sw_fd >= 0)
        return ioctl(sw_fd, EVIOCGSW(SW_BYTES), sw) >= 0;

    if(fake_path) {
        char buf[32] = "";
        FILE *f = fopen(fake_path, "r");
        if(f == NULL)
            return false;
        bool ok = fgets(buf, sizeof(buf), f) != NULL;
        fclose(f);
        if(!ok)
            return false;
        unsigned long word = strtoul(buf, NULL, 0);
        for(unsigned i = 0; i < SW_BYTES; i++)
            sw[i] = (uint8_t)(word >> (8 * i));
        return true;
    }

    return false;
}

/* Maps the current switch word onto control signals (the pure mapping
   lives in glowforge_switch_map.h, unit-tested on the host). */
static bool read_signals (control_signals_t *signals)
{
    uint8_t sw[SW_BYTES];

    if(!gfsw_read_raw(sw))
        return false;

    *signals = gfsw_map_bits(sw);

    return true;
}

void gfsw_init (void)
{
    control_signals_t initial = {0};

    /* Nothing on this hardware is an e-stop input. */
    hal.signals_cap.e_stop = Off;

    if((sw_fd = open(SWITCH_DEV, O_RDONLY | O_CLOEXEC)) < 0) {
        const char *p = getenv("GF_SWITCH_FILE");
        if(p != NULL && *p != '\0')
            fake_path = p;
    }

    if(!gfsw_available()) {
        /* Null-sink/host builds have no switch source: nothing to gate on. */
        hal.signals_cap.safety_door_ajar = Off;
        return;
    }

    hal.signals_cap.safety_door_ajar = On;

    if(read_signals(&initial))
        state = initial;

    if(state.safety_door_ajar)
        fflog(LOG_WARNING, "gfswitch: lid open or interlock loop open at startup");
}

control_signals_t gfsw_get_state (void)
{
    return gfsw_visible(state, state_get());
}

void gfsw_poll (void)
{
    control_signals_t now = {0}, want, on, off;

    if(!gfsw_available() || !read_signals(&now))
        return;

    state = now;

    /* What the core should currently see as asserted, given its state
       (the door signal is hidden while IDLE/JOG/HOMING - see gfsw_visible),
       diffed against what it has already been told. This also delivers a
       still-open door the moment the core leaves those states. Assertions
       and deassertions are reported separately: the core treats a
       deasserted report as clearing, and reads the door state back through
       get_state() to decide when to leave the door state. */
    want = gfsw_visible(now, state_get());
    delivered = gfsw_edges(want, delivered, &on, &off);

    if(on.bits)
        hal.control.interrupt_callback(on);

    if(off.bits)
        hal.control.interrupt_callback(off);
}
