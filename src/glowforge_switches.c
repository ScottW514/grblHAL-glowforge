/*
  glowforge_switches.c - safety switch inputs (lid, interlock, e-stop sense)

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

  Opening the lid mid-job therefore parks the job in the door state and closing
  it resumes, which is also what the hardware does to the beam - OK_2_FIRE
  drops the moment the chain opens, with or without this code.

  The e-stop bit (4) is a sense line whose resting state is ACTIVE on a healthy
  machine, and on the factory board it drops for the duration of any stepper
  motion and recovers at idle. Wiring it to the core's e_stop signal would
  therefore abort every job on this hardware, so it is opt-in through the
  shared machine settings and off by default - the same escape hatch the cloud
  client exposes as MOTION.ESTOP_HALTS_MOTION. When enabled, the assertion is
  the line dropping.

  Bit 6 (interlock latch tripped) is deliberately not gated on: its resting
  state on a healthy machine is not characterized, and a false assertion would
  wedge every job. The hardware chain enforces it regardless.
*/

#include "glowforge_switches.h"
#include "glowforge_switch_map.h"
#include "glowforge_io.h"

#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SWITCH_DEV        "/dev/input/event0"

static int sw_fd = -1;
static bool estop_gates_motion = false;
static control_signals_t state = {0};

/* Reads the switch device and maps it onto control signals (the pure
   mapping lives in glowforge_switch_map.h, unit-tested on the host).
   Leaves the previous state untouched if the device cannot be read, so
   a transient read failure cannot fake a door event. */
static bool read_signals (control_signals_t *signals)
{
    uint8_t sw[SW_BYTES] = {0};

    if(sw_fd < 0 || ioctl(sw_fd, EVIOCGSW(sizeof(sw)), sw) < 0)
        return false;

    *signals = gfsw_map_bits(sw, estop_gates_motion);

    return true;
}

void gfsw_init (void)
{
    control_signals_t initial = {0};

    /* Conf key mirrors the cloud client's MOTION.ESTOP_HALTS_MOTION. */
    estop_gates_motion = gfio_conf_read_float("estop_halts_motion", 0.0f) != 0.0f;

    if((sw_fd = open(SWITCH_DEV, O_RDONLY | O_CLOEXEC)) < 0) {
        /* Null-sink/host builds have no switch device: nothing to gate on. */
        hal.signals_cap.safety_door_ajar = Off;
        hal.signals_cap.e_stop = Off;
        return;
    }

    hal.signals_cap.safety_door_ajar = On;
    hal.signals_cap.e_stop = estop_gates_motion;

    if(read_signals(&initial))
        state = initial;

    if(state.safety_door_ajar)
        fprintf(stderr, "gfswitch: lid open or interlock loop open at startup\n");
    if(estop_gates_motion)
        fprintf(stderr, "gfswitch: e-stop sense gates motion (estop_halts_motion)\n");
}

control_signals_t gfsw_get_state (void)
{
    return state;
}

void gfsw_poll (void)
{
    control_signals_t now = {0}, changed;

    if(sw_fd < 0 || !read_signals(&now))
        return;

    if(now.bits == state.bits)
        return;

    changed.bits = now.bits ^ state.bits;
    state = now;

    /* Assertions and deassertions are reported separately: the core treats a
       deasserted report as clearing, and reads the door state back through
       get_state() to decide when to leave the door state. */
    if(changed.bits & now.bits)
        hal.control.interrupt_callback((control_signals_t){ .bits = changed.bits & now.bits });

    if(changed.bits & ~now.bits) {
        control_signals_t off = { .bits = changed.bits & ~now.bits };
        off.deasserted = On;
        hal.control.interrupt_callback(off);
    }
}
