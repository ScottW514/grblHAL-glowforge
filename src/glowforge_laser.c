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
       device), a soft reset aborts, the lid or the interlock loop
       opens (the job is cancelled: relock, alarm), or the timeout
       expires. A press with the lid open does not arm - the hardware
       button latch would not clear on it either.

  The armed window is JOB-based: it persists across the job (S changes
  and M5/M3 toggles do not re-prompt) and closes - relocking the
  latch - at program end (M2/M30/%), whenever the sender's connection
  changes (the consent belonged to the displaced session), after
  laser_disarm_s of spindle-off grace (counting down in Idle and in a
  job parked in Hold, Door or Tool Change - only a cycle, a jog or a
  lingering M3 keeps the window open), or immediately on alarm, homing,
  reset or a stream fault. The coolant fire gate is re-checked after
  the button wait, so a window can never open against a verdict that
  went bad during the wait. While unarmed or gated, fire requests are
  suppressed at the stream and reported.

  Gates, in order, at the first laser-on of a job: the coolant verdict
  (fire_ok), head presence (the head driver has probed - lens, air assist
  and beam detector are on the head), then the operator's button press.

  Config keys (shared machine config, re-read at each arm):
    laser_button_timeout_s   button wait budget (default 300; clamped
                             to 1-3600 - out-of-range values fall back
                             to the default, never wait-forever)
    laser_disarm_s           spindle-off grace before the armed window
                             closes (default 60)
    laser_power_model        density (dose as FIRE-bit density at full
                             duty) or analog (dose as PWM duty, the
                             default). Density wants $35 = 0: the floor
                             exists only because an analog duty cannot
                             go low without leaving the lasing band, and
                             it clamps the light end of the density
                             range for no gain.
    laser_pulse_ticks        density base period in machine ticks
                             (default 20 = 710 us at 28160 Hz)
    laser_pulse_min_ticks    shortest pulse the density model will emit
                             (default 3 = 106 us). Below it a period is
                             skipped and its debt carried, so a low
                             level arrives as fewer full-width pulses
                             rather than stubs the supply cannot strike.

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
#include "glowforge_switches.h"
#include "glowforge_switch_map.h"
#include "stepper_stream.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <fcntl.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define BUTTON_TIMEOUT_S_DEFAULT 300.0f
#define DISARM_S_DEFAULT         60.0f

/* Hardware PWM resolution: the power byte's 7 bits are written raw into
 * PWMSAR against a 127-count period (scope-verified 40 kHz carrier).
 * One definition, in stepper_stream.h - the shipper renders against it. */
#define PWM_PERIOD GF_PWM_PERIOD

/* Dose model. The tube draws current from ~3 % duty but does not lase
 * usefully below ~16 %, so an analog duty scaled by S walks into a dead
 * band at every corner and short segment. The density model instead
 * pins the duty at full and modulates how many ticks of each base
 * period fire, which cannot land in the band by construction. Default
 * period: 20 ticks of the 28160 Hz stream = 710 us, the factory's
 * ~1.43 kHz pulse rate. */
#define PULSE_TICKS_DEFAULT 20.0f

/* Shortest pulse worth emitting, in machine ticks. A 36 us stub (one
 * tick) draws no discharge at all on this supply, and the factory never
 * emits below one of its 100 us ticks; 3 ticks is 106 us. Below this a
 * period is skipped and its debt carried, so a low level arrives as
 * fewer full-width pulses rather than stubs. */
#define PULSE_MIN_TICKS_DEFAULT 3.0f
#define POWER_MODEL_KEY "laser_power_model"
#define PULSE_TICKS_KEY "laser_pulse_ticks"
#define PULSE_MIN_KEY   "laser_pulse_min_ticks"

/* Spindle state is written by the protocol thread (set_state) and read
 * by the stepper producer's segment updates and the poll below: kept in
 * an atomic byte so an arm edge is never seen half-written. */
static _Atomic uint8_t cur_state_value = 0;
static spindle_pwm_t spindle_pwm;
static bool hw_active;              /* GFSINK set: real device + button */
static _Atomic bool laser_ok = false;   /* armed window open */
static double disarm_at;            /* 0 = no grace running */
static unsigned armed_client_gen;   /* sender session the arm belongs to */
static _Atomic bool disarm_request = false; /* program end: close the window */
static _Atomic bool arming = false;         /* blocked in the button wait */
static double disarmed_at;          /* margin for the sample-window lag */
static double next_emission_check;  /* ~1 Hz witness pacing */
static on_program_completed_ptr on_program_completed;

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

/* One reading of the switches for the arm wait. A press only counts with
 * the lid closed and the interlock loop closed - the same condition under
 * which the hardware button latch would clear on it. */
enum { Arm_Waiting = 0, Arm_Pressed, Arm_LidOpen, Arm_InterlockOpen };

static int arm_switches (void)
{
    uint8_t sw[SW_BYTES];

    if(!gfsw_read_raw(sw))
        return Arm_Waiting;             /* unreadable: keep waiting */

    if(!gfsw_bit_set(sw, SW_BIT_DOORS))
        return Arm_LidOpen;
    if(gfsw_bit_set(sw, SW_BIT_INTERLOCK))
        return Arm_InterlockOpen;

    return gfsw_bit_set(sw, SW_BIT_BUTTON) ? Arm_Pressed : Arm_Waiting;
}

/* Pump the protocol while blocked in the button wait (same pattern as
 * the homing session). Returns false once the system is aborting. */
static bool pump (long timeout_us)
{
    serial_wait(timeout_us);
    serial_poll();
    return protocol_execute_realtime();
}

/* All latch writes go through the stream engine's serialized writer:
 * the shipper's run-start relight must be atomic against a concurrent
 * disarm here, or it can re-unlock a latch this module just locked. */
static void latch_lock (bool lock)
{
    gf_stream_laser_latch(lock);
}

bool gflaser_arming (void)
{
    return atomic_load(&arming);
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
    disarm_request = false;
    disarmed_at = wall_s();
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

    /* No head, no beam: the lens, air assist and beam detector live on
     * the head, and the hardware safety chain does not include head
     * presence. Presence is the head driver having probed - its sysfs
     * group exists - not the EV_SW head line. Checked before the latch
     * is unlocked and before the button ever lights. */
    if(hw_active) {
        char hall[16];
        if(gfio_rd_attr("head/hall_sensor", hall, sizeof(hall)) != 0) {
            report_message("laser fire blocked: no head detected", Message_Warning);
            system_raise_alarm(Alarm_AbortCycle);
            return false;
        }
    }

    /* Fan run profile + flow interrogation cover the whole armed
     * window, whatever the sender's M8/M9 state. */
    gfcool_laser_armed(true);
    latch_lock(false);

    if(gfsw_available()) {
        /* The wait runs with the latch unlocked, so it must always be
         * bounded: out-of-range values (including 0 and garbage that
         * parses negative or NaN) fall back to the default. */
        float timeout_s = gfio_conf_read_float("laser_button_timeout_s", BUTTON_TIMEOUT_S_DEFAULT);
        if(!(timeout_s >= 1.0f && timeout_s <= 3600.0f))
            timeout_s = BUTTON_TIMEOUT_S_DEFAULT;
        double deadline = wall_s() + (double)timeout_s;

        button_led(255);
        report_message("press the button to start the laser job", Message_Info);

        /* The lid and the interlock loop are checked here explicitly:
         * the wait runs at Idle, where the door signal is hidden from
         * the core, and an open lid or loop cancels the job outright
         * (the factory does the same; the hardware button latch sets on
         * the lid and would ignore a press anyway). */
        bool pressed = false, aborted = false;
        arming = true;
        while(!pressed && !aborted) {
            switch(arm_switches()) {
                case Arm_Pressed:
                    pressed = true;
                    gfsw_button_consumed();   /* not a pause press */
                    break;
                /* Lid or loop open: the job is cancelled the way a mid-job
                 * open cancels it - a soft reset from a standstill (the
                 * position is kept, no alarm), so the sender's stream ends
                 * on the banner and nothing needs unlocking afterward. The
                 * reset lands in the pump below, which then returns false. */
                case Arm_LidOpen:
                    report_message("lid opened during arm - job cancelled", Message_Warning);
                    protocol_enqueue_realtime_command(CMD_RESET);
                    aborted = !pump(50000);
                    break;
                case Arm_InterlockOpen:
                    report_message("interlock open during arm - job cancelled", Message_Warning);
                    protocol_enqueue_realtime_command(CMD_RESET);
                    aborted = !pump(50000);
                    break;
                default:
                    if(!pump(50000))
                        aborted = true;             /* soft reset during the wait */
                    else if(wall_s() > deadline) {
                        report_message("laser arm timed out waiting for the button", Message_Warning);
                        system_raise_alarm(Alarm_AbortCycle);
                        aborted = true;
                    }
                    break;
            }
        }
        arming = false;
        button_led(0);

        if(aborted) {
            latch_lock(true);
            gfcool_laser_armed(false);
            return false;
        }
    }

    /* Re-check the coolant gate at the moment it matters: the wait can
     * run for minutes, and the verdict may have gone bad (or stale)
     * during it. */
    if(!gfcool_fire_ok()) {
        latch_lock(true);
        gfcool_laser_armed(false);
        report_message("laser fire blocked: coolant flow fault or over-temperature", Message_Warning);
        system_raise_alarm(Alarm_AbortCycle);
        return false;
    }

    /* Select the dose model for this window, before any fire reaches
     * the stream. */
    char model[16] = "";
    bool density = gfio_conf_read(POWER_MODEL_KEY, model, sizeof(model)) == 0 &&
                    strcmp(model, "density") == 0;
    if(density) {
        float ticks = gfio_conf_read_float(PULSE_TICKS_KEY, PULSE_TICKS_DEFAULT);
        if(!(ticks >= 1.0f))            /* also catches NaN */
            ticks = PULSE_TICKS_DEFAULT;
        float min_ticks = gfio_conf_read_float(PULSE_MIN_KEY, PULSE_MIN_TICKS_DEFAULT);
        if(!(min_ticks >= 1.0f))
            min_ticks = PULSE_MIN_TICKS_DEFAULT;
        gf_stream_laser_model((uint32_t)ticks, (uint32_t)min_ticks);
        if(settings.pwm_spindle.pwm_min_value > 0.0f)
            report_message("$35 floors the density model; set $35=0 for its full range",
                            Message_Warning);
    } else
        gf_stream_laser_model(0, 0);

    disarm_request = false;
    laser_ok = true;
    disarm_at = 0.0;
    armed_client_gen = serial_client_generation();
    gf_stream_laser_arm(true);
    report_message(density ? "laser armed (density)" : "laser armed", Message_Info);

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
    (void)spindle;

    spindle_state_t cur = { .value = atomic_load(&cur_state_value) };

    if(state.on && !cur.on && !laser_ok && !gflaser_arm())
        state.on = Off;                 /* refused/aborted: stay dark */

    atomic_store(&cur_state_value, state.value);

    /* The synchronous path carries the speed. Per-segment updates cover
     * a laser block, but an S word executed between blocks - with the
     * planner drained, so nothing is streaming - only arrives here, and
     * dropping it leaves the next move cutting at the previous level.
     * Duty only: fire stays where spindleUpdatePWM and its gates put it. */
    if(state.on && spindle_pwm.compute_value) {
        uint_fast16_t pwm = spindle_pwm.compute_value(&spindle_pwm, rpm, false);
        gf_stream_laser_power((uint8_t)(pwm > PWM_PERIOD ? PWM_PERIOD : pwm));
    }
}

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    (void)spindle;
    return (spindle_state_t){ .value = atomic_load(&cur_state_value) };
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

    /* Emission witness (~1 Hz): laser_on_sampled counts the last ~1 s
     * window's emitting samples on the gated output of the hardware
     * AND-gate - physical evidence, not a commanded state. Emission
     * while the window is closed (3 s of margin covers the sample
     * window's lag past a disarm) is never legitimate. */
    if(hw_active) {
        double t = wall_s();
        if(t >= next_emission_check) {
            next_emission_check = t + 1.0;
            char v[8] = "";
            if(!laser_ok && (disarmed_at == 0.0 || t - disarmed_at > 3.0) &&
               gfio_rd_attr("cnc/laser_on_sampled", v, sizeof(v)) == 0 &&
               atoi(v) > 0) {
                latch_lock(true);
                report_message("uncommanded laser emission sensed - latch relocked", Message_Warning);
                system_raise_alarm(Alarm_AbortCycle);
            }
        }
    }

    if(!laser_ok)
        return;

    sys_state_t st = state_get();

    if(st & (STATE_ALARM | STATE_ESTOP | STATE_HOMING)) {
        gflaser_disarm();
        return;
    }

    /* A sender change closes the window: the button press that armed
     * it belonged to the displaced session. */
    if(serial_client_generation() != armed_client_gen) {
        gflaser_disarm();
        return;
    }

    /* Program end closes the window too - the consent is spent with
     * the job. The relock waits for the kernel to finish the queue
     * tail, so the request stays pending until the device is idle. */
    if(disarm_request) {
        char state[16] = "";
        if(!hw_active || (gfio_rd_attr("cnc/state", state, sizeof(state)) == 0 &&
                           strcmp(state, "idle") == 0)) {
            gflaser_disarm();
            return;
        }
    }

    /* Otherwise the window closes after a spindle-off grace. The grace
     * counts down whenever the spindle is off - including a job parked
     * in Hold, Door or Tool Change - and only a cycle, a jog or a
     * lingering M3 keeps the window open. */
    spindle_state_t cur = { .value = atomic_load(&cur_state_value) };
    if(cur.on || (st & (STATE_CYCLE | STATE_JOG))) {
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

/* Program end (M2/M30/%): the core calls this on the protocol thread
 * after protocol_buffer_synchronize, so the planner is drained; the
 * kernel ring tail may still be playing, hence the deferred close in
 * gflaser_poll. */
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(!check_mode && laser_ok)
        disarm_request = true;

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

void gflaser_init (void)
{
    const char *dev = getenv("GFSINK");
    hw_active = dev != NULL && *dev != '\0';

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;

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
