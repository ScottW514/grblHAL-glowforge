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
  for the status panel, and the laser arm flow reads the button through
  gfsw_read_raw().

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

  What a job does when one of them opens is the factory firmware's policy
  (lid_policy = cancel, the default): the core parks the job in the door
  state - a planned deceleration with the laser off and the position kept -
  and the moment it is parked the job is CANCELLED: the armed window closes,
  the reason is reported, a soft reset ends the sender's stream (the
  position is not lost - the reset comes from a fully parked state, so no
  alarm), and the head returns on its own to where the job started, lid
  open or not. The next job re-arms with a fresh button press, which is
  the same press the hardware button latch (set by the lid) needs. With
  lid_policy = hold the door parks the job and a cycle start resumes it
  once the lid is closed - stock grblHAL behavior; the hardware still needs
  a button press before the beam returns.

  While the core is IDLE, JOG or HOMING the door signal is deliberately
  hidden from it (gfsw_visible, applied to both get_state() and the edge
  delivery): the lid is opened at idle every time material is loaded, the
  beam is blocked in hardware anyway, and a door seen while idle - at boot,
  on a stop, or as an edge - strands the controller in Door until a cycle
  start. It is also hidden during the return-to-start motion after a
  cancel, which runs with the latch locked. The signal becomes visible, and
  is delivered, the moment the core is in any other state, so a job started
  with the lid open parks (and cancels) on the first poll.

  The button (bit 2) is the operator's consent in the arm flow (the wait
  reads it through gfsw_read_raw()) and, outside the arm wait, the job
  pause/resume toggle: a press while a job runs is a feed hold, a press
  while it is held is a cycle start. A hold has no further meaning.

  Bit 4 (hv_enable) is the readback of the board's HV_ENABLE output, not an
  input: it is low at idle and high only while a run feeds the charge-pump
  watchdog with the lid closed. It is telemetry (forgectrl shows it) and gates
  nothing here; the core's e_stop signal is not wired to anything on this
  hardware.

  Bit 6 (interlock latch tripped) is deliberately not gated on: its resting
  state on a healthy machine is not characterized, and a false assertion would
  wedge every job. The hardware chain enforces it regardless.

  Test hook: with GF_SWITCH_FILE set (and no device, i.e. a null-sink host
  build), the EV_SW word is read from that file instead - an integer,
  decimal or 0x-hex, holding the bitmask exactly as EVIOCGSW would return
  it. The host harnesses use it to open the lid, break the interlock loop
  and press the button against the real gating code.
*/

#include "fflog.h"
#include "glowforge_switches.h"
#include "glowforge_switch_map.h"
#include "glowforge_io.h"
#include "glowforge_laser.h"

#include "grbl/hal.h"
#include "grbl/planner.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define SWITCH_DEV        "/dev/input/event0"

/* Config key: what an open lid / interlock loop does to a running job.
   "cancel" (default) = the factory's abort + return to the job start;
   "hold" = stock grblHAL door hold, cycle start resumes. Re-read at each
   door event, so a change applies to the next job. */
#define LID_POLICY_KEY    "lid_policy"

/* Give the return-to-start park a moment to be accepted / to start. */
#define PARK_ENQUEUE_RETRY_S   2.0
#define PARK_ZERO_LENGTH_S     1.0
#define DRAIN_WAIT_S           2.0   /* kernel drain of the hold tail (~depth) */

static int sw_fd = -1;
static const char *fake_path;              /* GF_SWITCH_FILE (host tests) */
static control_signals_t state = {0};      /* raw reading of the switch device */
static control_signals_t delivered = {0};  /* asserted signals the core has been told about */
static uint8_t raw[SW_BYTES];              /* last raw word (button, reason strings) */
static bool button_prev;                   /* button level at the previous poll */

/* The cancel policy's state (see the file header). */
static enum {
    Cancel_None = 0,       /* nothing pending */
    Cancel_WaitDrain,      /* job cancelled; the kernel drains the hold tail, then reset */
    Cancel_ResetSent,      /* job cancelled, soft reset queued; wait for Idle */
    Cancel_ParkQueued      /* return-to-start motion enqueued; wait for it to end */
} cancel_state = Cancel_None;
static bool door_hidden;                   /* hide the door through reset + park */
static double park_at;                     /* wall time the park was enqueued / retried from */
static bool park_saw_cycle;
static bool hold_seen;                     /* lid_policy=hold: parked, policy already read */
static bool have_job_start;
static float job_start[N_AXIS];            /* machine position when the job began */
static bool job_active;                    /* a job is under way (running or held) */
static sys_state_t prev_state = STATE_IDLE;
static on_state_change_ptr on_state_change_chain;

static double wall_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

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
    if(!gfsw_read_raw(raw))
        return false;

    *signals = gfsw_map_bits(raw);

    return true;
}

static bool lid_cancels (void)
{
    char v[16] = "";
    if(gfio_conf_read(LID_POLICY_KEY, v, sizeof(v)) == 0 && strcmp(v, "hold") == 0)
        return false;
    return true;
}

/* The core's view of the door: the pure visibility policy, plus the
   cancel path's own hiding through the reset and the return-to-start
   motion (both run with the latch locked; a door seen there would
   re-park the park). */
static control_signals_t visible (control_signals_t now)
{
    now = gfsw_visible(now, state_get());
    if(door_hidden)
        now.safety_door_ajar = Off;
    return now;
}

/* Job start = the machine position at the Idle -> Cycle transition that
   begins a job (not a jog, not the park itself): where a cancelled job's
   head returns to, as the factory returns to its job origin.

   A job is under way from that transition until the core is Idle with
   the planner empty (the program ran out, a stop, a reset) or in an
   alarm. The core restarts a held cycle through Idle with the planner
   still loaded, so a pause and resume is the same job and keeps its
   start; a job abandoned in a hold and reset is over. */
static void onStateChange (sys_state_t new_state)
{
    if(new_state == STATE_IDLE) {
        if(plan_get_current_block() == NULL)
            job_active = false;
    } else if(new_state & (STATE_ALARM|STATE_ESTOP|STATE_SLEEP))
        job_active = false;
    else if(new_state == STATE_CYCLE && prev_state == STATE_IDLE && !job_active) {
        if(cancel_state == Cancel_None) {
            int32_t steps[N_AXIS];
            memcpy(steps, sys.position, sizeof(steps));
            system_convert_array_steps_to_mpos(job_start, steps);
            have_job_start = true;
        }
        job_active = true;
    }
    prev_state = new_state;

    if(on_state_change_chain)
        on_state_change_chain(new_state);
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

    on_state_change_chain = grbl.on_state_change;
    grbl.on_state_change = onStateChange;

    if(!gfsw_available()) {
        /* Null-sink/host builds have no switch source: nothing to gate on. */
        hal.signals_cap.safety_door_ajar = Off;
        return;
    }

    hal.signals_cap.safety_door_ajar = On;

    if(read_signals(&initial))
        state = initial;
    button_prev = gfsw_bit_set(raw, SW_BIT_BUTTON);

    if(state.safety_door_ajar)
        fflog(LOG_WARNING, "gfswitch: lid open or interlock loop open at startup");
}

control_signals_t gfsw_get_state (void)
{
    return visible(state);
}

/* --- the button: pause / resume toggle outside the arm wait ------------- */

void gfsw_button_consumed (void)
{
    button_prev = true;
}

static void button_edge (sys_state_t st)
{
    if(gflaser_arming())
        return;                     /* that press is the arm's consent */

    if(st == STATE_CYCLE) {
        protocol_enqueue_realtime_command(CMD_FEED_HOLD);
        report_message("button pressed - job paused", Message_Info);
    } else if(st == STATE_HOLD) {
        protocol_enqueue_realtime_command(CMD_CYCLE_START);
        report_message("button pressed - job resumed", Message_Info);
    }
    /* Idle, jog, homing, door, alarm: a press means nothing here. */
}

/* --- the door: cancel policy --------------------------------------------- */

static bool door_parked (sys_state_t st)
{
    return st == STATE_SAFETY_DOOR &&
            (sys.parking_state == Parking_DoorAjar || sys.parking_state == Parking_DoorClosed);
}

static void cancel_job (void)
{
    /* Latch first: FIRE is severed before anything else happens (the
       hold already turned the spindle off; this closes the armed window
       and relocks the kernel latch). */
    gflaser_disarm();

    const char *why = gfsw_bit_set(raw, SW_BIT_INTERLOCK) && gfsw_bit_set(raw, SW_BIT_DOORS)
                       ? "interlock open" : "lid opened";
    char msg[96];
    snprintf(msg, sizeof(msg), "%s - job cancelled%s", why,
             have_job_start ? ", returning to the job start" : "");
    report_message(msg, Message_Warning);
    fflog(LOG_NOTICE, "gfswitch: %s", msg);

    /* Hide the door from here on: the reset's re-init and the park must
       not see it. Cleared when the park has ended. */
    door_hidden = true;
    cancel_state = Cancel_WaitDrain;
    park_at = wall_s();
}

/* The kernel is still playing the queued tail of the hold's deceleration
   for up to a stream depth after the core parks; the reset waits for it
   (bounded), so nothing stops it short and the park runs on an idle
   kernel. Without a device (null-sink) there is nothing to wait for. */
static bool kernel_idle (void)
{
    char state[16] = "";
    if(gfio_rd_attr("cnc/state", state, sizeof(state)) != 0)
        return true;
    return strcmp(state, "idle") == 0;
}

static void send_reset (void)
{
    cancel_state = Cancel_ResetSent;
    park_at = wall_s();
    /* A soft reset from a fully parked door with the kernel idle: no
       motion is in flight, so the core keeps the position (no alarm 3);
       the sender's stream is flushed and it sees the reset banner - the
       job is over for it. */
    protocol_enqueue_realtime_command(CMD_RESET);
}

static void cancel_poll (sys_state_t st)
{
    switch(cancel_state) {

        case Cancel_None:
            /* The policy is read once per door event, when the job has
               just parked; under "hold" it then stays parked (stock
               behavior) without re-reading the config every pass. */
            if(door_parked(st)) {
                if(!hold_seen) {
                    if(lid_cancels())
                        cancel_job();
                    else
                        hold_seen = true;
                }
            } else
                hold_seen = false;
            break;

        case Cancel_WaitDrain:
            /* Idle kernel, the bounded wait, or the core leaving the door
               state under us (a cycle start; the latch is already locked):
               the job is cancelled either way - reset now. */
            if(kernel_idle() || st != STATE_SAFETY_DOOR || wall_s() - park_at > DRAIN_WAIT_S)
                send_reset();
            break;

        case Cancel_ResetSent:
            if(sys.reset_pending || st != STATE_IDLE) {
                if(st & (STATE_ALARM | STATE_ESTOP)) {
                    cancel_state = Cancel_None;     /* something else took over */
                    door_hidden = false;
                }
                break;
            }
            if(!have_job_start) {
                cancel_state = Cancel_None;
                door_hidden = false;
                break;
            }
            {
                char cmd[64];
                snprintf(cmd, sizeof(cmd), "G53G0X%.3fY%.3fZ%.3f",
                         job_start[X_AXIS], job_start[Y_AXIS], job_start[Z_AXIS]);
                if(grbl.enqueue_gcode(cmd)) {
                    cancel_state = Cancel_ParkQueued;
                    park_at = wall_s();
                    park_saw_cycle = false;
                } else if(wall_s() - park_at > PARK_ENQUEUE_RETRY_S) {
                    report_message("return to the job start could not be queued", Message_Warning);
                    cancel_state = Cancel_None;
                    door_hidden = false;
                }
            }
            break;

        case Cancel_ParkQueued:
            if(st == STATE_CYCLE)
                park_saw_cycle = true;
            else if(st & (STATE_ALARM | STATE_ESTOP)) {
                cancel_state = Cancel_None;
                door_hidden = false;
            } else if(st == STATE_IDLE && (park_saw_cycle || wall_s() - park_at > PARK_ZERO_LENGTH_S)) {
                /* Parked at the job start (or it was already there). */
                report_message("returned to the job start", Message_Info);
                cancel_state = Cancel_None;
                door_hidden = false;
            }
            break;
    }
}

void gfsw_poll (void)
{
    control_signals_t now = {0}, want, on, off;
    sys_state_t st = state_get();

    if(!gfsw_available() || !read_signals(&now))
        return;

    state = now;

    /* Button edge: pause / resume, outside the arm wait. */
    bool button = gfsw_bit_set(raw, SW_BIT_BUTTON);
    if(button && !button_prev)
        button_edge(st);
    button_prev = button;

    /* What the core should currently see as asserted, given its state
       (the door signal is hidden while IDLE/JOG/HOMING and through a
       cancel - see visible()), diffed against what it has already been
       told. This also delivers a still-open door the moment the core
       leaves those states. Assertions and deassertions are reported
       separately: the core treats a deasserted report as clearing, and
       reads the door state back through get_state() to decide when to
       leave the door state. */
    want = visible(now);
    delivered = gfsw_edges(want, delivered, &on, &off);

    if(on.bits)
        hal.control.interrupt_callback(on);

    if(off.bits)
        hal.control.interrupt_callback(off);

    cancel_poll(st);
}
