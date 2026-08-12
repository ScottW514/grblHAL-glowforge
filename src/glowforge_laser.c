/*
  glowforge_laser.c - laser spindle + operator arming for the Glowforge board

  Part of grblHAL-glowforge. Maps grblHAL's laser spindle (M3/M4 + S
  words, $32 laser mode) onto the pulse stream: S values become raw
  7-bit power bytes (the kernel writes them straight into PWMSAR against
  a 127-count period, so the spindle PWM is precomputed to a period of
  exactly 127), and fire rides the per-tick FIRE bit. Power lands via
  the core's per-segment spindle update, which the stepper producer runs
  at exact virtual-tick positions - so the laser only ever fires inside
  motion segments of laser blocks. Jogs, G0 and homing are fire-free by
  construction, and the kernel's end-of-data backstop forces the lines
  low whenever a stream ends.

  ARMING - the operator stays in the loop. FIRE reaches the tube only
  through the hardware AND chain (lid switches, interlock, HV OK, the
  charge-pump watchdog the kernel feeds while a run plays, and the
  BUTTON latch). On the first laser-on of a job this module:

    1. refuses outright while a coolant fire gate is active (flow FAULT
       or over-temperature - glowforge_cooling.c owns the verdicts),
    2. forces the run fan profile on (so flow interrogation covers
       every fire window) and unlocks the kernel laser latch,
    3. lights the button white and BLOCKS the gcode stream - pumping
       real-time traffic like the homing session does - until the
       operator presses the physical button (EV_SW bit 2 on the input
       device), a soft reset aborts, or the timeout expires.

  The armed window then persists across the job (S changes and M5/M3
  toggles do not re-prompt) and closes - relocking the latch - after
  laser_disarm_s of spindle-off idle, or immediately on alarm, homing,
  reset or a stream fault. While unarmed or gated, fire requests are
  suppressed at the stream and reported.

  Config keys (shared machine config, re-read at each arm):
    laser_button_timeout_s   button wait budget (default 300; 0 = no
                             timeout, wait until pressed or aborted)
    laser_disarm_s           spindle-off idle grace before the armed
                             window closes (default 60)

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "driver.h"
#include "glowforge_laser.h"
#include "glowforge_cooling.h"
#include "glowforge_io.h"
#include "stepper_stream.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <fcntl.h>
#include <linux/input.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* EV_SW bits per the device tree: 2 = the big button. */
#define SWITCH_DEV     "/dev/input/event0"
#define SW_BIT_BUTTON  2

#define BUTTON_TIMEOUT_S_DEFAULT 300.0f
#define DISARM_S_DEFAULT         60.0f

/* Hardware PWM resolution: the power byte's 7 bits are written raw into
 * PWMSAR against a 127-count period (scope-verified 40 kHz carrier). */
#define PWM_PERIOD 127

static spindle_state_t cur_state = {0};
static spindle_pwm_t spindle_pwm;
static bool hw_active;              /* GFSINK set: real device + button */
static _Atomic bool laser_ok = false;   /* armed window open */
static double disarm_at;            /* 0 = no grace running */

/* Producer-thread fire suppression, reported from the protocol thread. */
enum { Suppress_None = 0, Suppress_Unarmed, Suppress_Coolant };
static _Atomic int suppressed = Suppress_None;

static double wall_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static void button_led (uint32_t val)
{
    char path[64], v[16];
    snprintf(v, sizeof(v), "%u", val);
    for(uint32_t led = 1; led <= 3; led++) {
        snprintf(path, sizeof(path), "/sys/class/leds/button_led_%u/target", led);
        int fd = open(path, O_WRONLY);
        if(fd >= 0) {
            if(write(fd, v, strlen(v)) < 0) { /* LED only; ignore */ }
            close(fd);
        }
    }
}

static bool button_pressed (int fd)
{
    uint8_t sw[2] = {0};
    if(fd < 0 || ioctl(fd, EVIOCGSW(sizeof(sw)), sw) < 0)
        return false;
    return !!(sw[SW_BIT_BUTTON / 8] & (1u << (SW_BIT_BUTTON % 8)));
}

/* Pump the protocol while blocked in the button wait (same pattern as
 * the homing session). Returns false once the system is aborting. */
static bool pump (long timeout_us)
{
    serial_wait(timeout_us);
    serial_poll();
    return protocol_execute_realtime();
}

static void latch_lock (bool lock)
{
    gfio_wr_attr("cnc/laser_latch", lock ? "1" : "0");
}

void gflaser_disarm (void)
{
    if(!laser_ok)
        return;

    laser_ok = false;
    gf_stream_laser_arm(false);
    latch_lock(true);
    button_led(0);
    gfcool_laser_armed(false);
    disarm_at = 0.0;
    report_message("laser disarmed - latch locked", Message_Info);
}

/* The first laser-on of a job: gate, unlock, wait for the operator's
 * button press. Runs on the protocol thread with the planner synced
 * (the core syncs every spindle state change, laser mode included). */
static bool gflaser_arm (void)
{
    if(!gfcool_fire_ok()) {
        report_message("laser fire blocked: coolant flow fault or over-temperature", Message_Warning);
        system_raise_alarm(Alarm_AbortCycle);
        return false;
    }

    /* Fan run profile + flow interrogation cover the whole armed
     * window, whatever the sender's M8/M9 state. */
    gfcool_laser_armed(true);
    latch_lock(false);

    if(hw_active) {
        float timeout_s = gfio_conf_read_float("laser_button_timeout_s", BUTTON_TIMEOUT_S_DEFAULT);
        double deadline = timeout_s > 0.0f ? wall_s() + (double)timeout_s : 0.0;
        int fd = open(SWITCH_DEV, O_RDONLY | O_NONBLOCK);

        button_led(255);
        report_message("press the button to start the laser job", Message_Info);

        bool pressed = false, aborted = false;
        while(!pressed && !aborted) {
            pressed = button_pressed(fd);
            if(!pressed) {
                if(!pump(50000))
                    aborted = true;             /* soft reset during the wait */
                else if(deadline != 0.0 && wall_s() > deadline) {
                    report_message("laser arm timed out waiting for the button", Message_Warning);
                    system_raise_alarm(Alarm_AbortCycle);
                    aborted = true;
                }
            }
        }
        if(fd >= 0)
            close(fd);
        button_led(0);

        if(aborted) {
            latch_lock(true);
            gfcool_laser_armed(false);
            return false;
        }
    }

    laser_ok = true;
    disarm_at = 0.0;
    gf_stream_laser_arm(true);
    report_message("laser armed", Message_Info);

    return true;
}

/* --- spindle backends ----------------------------------------------------- */

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    /* Precompute against a clock that makes one PWM period exactly the
     * hardware's 127 counts, so computed values are raw power bytes. */
    spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle,
                                   (uint32_t)((float)PWM_PERIOD * settings.pwm_spindle.pwm_freq));

    return true;
}

static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    (void)spindle; (void)rpm;

    if(state.on && !cur_state.on && !laser_ok && !gflaser_arm())
        state.on = Off;                 /* refused/aborted: stay dark */

    cur_state = state;
}

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    (void)spindle;
    return cur_state;
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    (void)spindle;
    return spindle_pwm.compute_value(&spindle_pwm, rpm, false);
}

/* Runs on the stepper producer thread (per-segment, under the core
 * lock) as well as the protocol thread (state changes, overrides) - no
 * reporting from here, only the deferred suppression note. */
static void spindleUpdatePWM (spindle_ptrs_t *spindle, uint_fast16_t pwm)
{
    (void)spindle;

    bool fire = pwm != spindle_pwm.off_value;

    if(fire && !laser_ok) {
        fire = false;
        pwm = spindle_pwm.off_value;
        suppressed = Suppress_Unarmed;
    } else if(fire && !gfcool_fire_ok()) {
        fire = false;
        pwm = spindle_pwm.off_value;
        suppressed = Suppress_Coolant;
    }

    gf_stream_laser((uint8_t)(pwm > PWM_PERIOD ? PWM_PERIOD : pwm), fire);
}

/* --- lifecycle ------------------------------------------------------------ */

void gflaser_poll (void)
{
    int note = atomic_exchange(&suppressed, Suppress_None);
    if(note != Suppress_None)
        report_message(note == Suppress_Coolant
                        ? "laser fire suppressed: coolant flow fault or over-temperature"
                        : "laser fire suppressed: laser not armed", Message_Warning);

    if(!laser_ok)
        return;

    sys_state_t st = state_get();

    if(st & (STATE_ALARM | STATE_ESTOP | STATE_HOMING)) {
        gflaser_disarm();
        return;
    }

    /* The armed window closes after a spindle-off idle grace; any job
     * activity (or a lingering M3) keeps it open. */
    if(cur_state.on || !(st == STATE_IDLE || st == STATE_CHECK_MODE)) {
        disarm_at = 0.0;
        return;
    }

    double now = wall_s();
    if(disarm_at == 0.0)
        disarm_at = now + (double)gfio_conf_read_float("laser_disarm_s", DISARM_S_DEFAULT);
    else if(now >= disarm_at) {
        /* Never relock while the kernel still plays a queue tail - a
         * severed FIRE there would truncate the job's last bytes. The
         * grace makes this unreachable in practice; check anyway. */
        char state[16] = "";
        if(!hw_active || (gfio_rd_attr("cnc/state", state, sizeof(state)) == 0 &&
                           strcmp(state, "idle") == 0))
            gflaser_disarm();
        else
            disarm_at = now + 1.0;
    }
}

void gflaser_init (void)
{
    const char *dev = getenv("GFSINK");
    hw_active = dev != NULL && *dev != '\0';

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.direction = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleUpdatePWM,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    spindle_register(&spindle, "Glowforge laser");
}
