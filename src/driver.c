/*
  driver.c - grblHAL driver for the Glowforge factory board (Linux userspace)

  Part of grblHAL-glowforge. HAL-vector shape descended from the grblHAL
  Simulator driver (Copyright (c) 2020-2026 Terje Io); every emulated-MCU
  backend is replaced with the real machine: steps stream into the
  glowforge.ko SDMA pulse ring (stepper_stream.c), IO goes through the
  kernel driver's sysfs (glowforge_io.c), settings persist to a file
  (eeprom.c), and the Grbl protocol runs over TCP/stdio (serial.c).

  Concurrency model: a single recursive "core" mutex stands in for
  interrupt masking. hal.irq_disable/enable map to it, the atomics
  helpers take it, the stepper producer thread holds it for every core
  stepper interrupt callback, and the hal.stepper entry points here take
  it so foreground calls (mc_reset's st_go_idle etc.) serialize against a
  callback in flight. The atomics MUST use this lock rather than
  C11 atomics: the core's ISR path does direct non-atomic RMW on
  sys.rt_exec_state, which only the shared lock serializes.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver.h"
#include "serial.h"
#include "stepper_stream.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

static pthread_mutex_t core_mx = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;

static on_execute_realtime_ptr on_execute_realtime;
static driver_reset_ptr driver_reset_chain;
static coolant_state_t coolant_state = {0};
static _Atomic bool exit_requested = false;

/* Deferred hal.delay_ms callback (fired from the realtime hook; the core
 * itself only ever uses the blocking form, plugins may not). */
static void (*volatile delay_callback)(void) = NULL;
static volatile uint32_t delay_deadline_ms;

void gf_core_lock (void)
{
    pthread_mutex_lock(&core_mx);
}

void gf_core_unlock (void)
{
    pthread_mutex_unlock(&core_mx);
}

void driver_request_exit (void)
{
    exit_requested = true;
}

static uint32_t millis (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static uint64_t micros (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static void sleep_us (long us)
{
    struct timespec ts = { .tv_sec = 0, .tv_nsec = us * 1000 };
    nanosleep(&ts, NULL);
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(ms > 0) {
        if(callback) {
            delay_deadline_ms = millis() + ms;
            delay_callback = callback;
        } else {
            /* Blocking delay on the protocol thread. RX is polled, not
             * interrupt-driven, so keep polling here or a G4/tool-change
             * wait would blind real-time commands for its whole span. */
            uint32_t until = millis() + ms;
            while((int32_t)(until - millis()) > 0) {
                serial_poll();
                sleep_us(1000);
            }
        }
    } else if(callback)
        callback();
    else
        delay_callback = NULL;   /* (0, NULL) cancels a pending callback */
}

/* --- stepper: thin, core-locked wrappers over the stream engine ---------- */

static void stepperWakeUp (void)
{
    gf_core_lock();
    gf_stream_wakeup();
    gf_core_unlock();
}

static void stepperGoIdle (bool clear_signals)
{
    (void)clear_signals;   /* pulse bytes have no persistent pin state */

    gf_core_lock();
    gf_stream_go_idle();
    gf_core_unlock();
}

static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    gf_core_lock();
    gf_stream_cycles_per_tick(cycles_per_tick);
    gf_core_unlock();
}

static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits)
        stepper->dir_changed.bits = 0;   /* direction rides in every pulse byte */

    if(stepper->step_out.bits)
        gf_stream_pulse((uint8_t)stepper->step_out.bits, (uint8_t)stepper->dir_out.bits);
    /* NOTE: $2/$3 step/dir invert masks are intentionally NOT applied: the
     * pulse-byte direction semantics are fixed by the kernel contract and
     * hardware-verified. Axis orientation is a homing-milestone concern. */
}

static void stepperEnable (axes_signals_t enable, bool hold)
{
    (void)enable; (void)hold;
    /* No-op toward the kernel: cnc stays enabled for the process lifetime
     * (cycling enable/disable would fight the kernel state machine) and
     * torque control is the PIC run/hold current scheme in the stream. */
}

/* --- minimal signal backends (real inputs are later milestones) ---------- */

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    (void)on; (void)homing_cycle;
}

static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};
    return signals;
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};
    return signals;
}

static void coolantSetState (coolant_state_t mode)
{
    coolant_state = mode;
}

static coolant_state_t coolantGetState (void)
{
    return coolant_state;
}

/* --- atomics: serialized by the core lock (see file header) -------------- */

static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    gf_core_lock();
    *ptr |= bits;
    gf_core_unlock();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    gf_core_lock();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    gf_core_unlock();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    gf_core_lock();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    gf_core_unlock();
    return prev;
}

static void irqDisable (void)
{
    gf_core_lock();
}

static void irqEnable (void)
{
    gf_core_unlock();
}

/* ------------------------------------------------------------------------- */

void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    (void)settings; (void)changed;
    /* Must exist: the core's wrapper calls it without a NULL check. No
     * driver-side settings consumers yet (spindle/laser is a later
     * milestone). */
}

static void driverReset (void)
{
    /* Called for both EXEC_STOP and EXEC_RESET; only a real reset may kill
     * the stream (EXEC_STOP's controlled decel must play out normally). */
    if(sys.reset_pending)
        gf_stream_reset();

    driver_reset_chain();
}

/* Realtime hook, chained on grbl.on_execute_realtime: everything the
 * driver must do periodically on the protocol thread. Also paces the
 * loop - without a sleep the protocol thread busy-spins and starves the
 * rest of the single-core i.MX6. */
static void glowforge_process_realtime (uint_fast16_t state)
{
    serial_poll();

    if(delay_callback && (int32_t)(millis() - delay_deadline_ms) >= 0) {
        void (*cb)(void) = delay_callback;
        delay_callback = NULL;
        cb();
    }

    if(gf_stream_fault_take()) {
        fprintf(stderr, "gfstream: stream fault - raising alarm, re-home required\n");
        system_raise_alarm(Alarm_MotorFault);
    }

    if(exit_requested && state != STATE_CYCLE && state != STATE_JOG && state != STATE_HOMING)
        exit(EXIT_SUCCESS);

    sleep_us(state == STATE_IDLE || state == STATE_ALARM ? 1000 : 200);

    on_execute_realtime(state);
}

bool driver_setup (settings_t *settings)
{
    settings_changed_flags_t changed_flags = {0};
    hal.settings_changed(settings, changed_flags);
    hal.stepper.go_idle(true);
    hal.coolant.set_state((coolant_state_t){0});

    return settings->version.id == 23;
}

bool driver_init (void)
{
    gf_stream_init();

    hal.info = "Glowforge";
    hal.driver_version = "260802";
    hal.driver_url = "https://github.com/ScottW514/grblHAL-glowforge";
    hal.board = "Glowforge factory control board (i.MX6)";
    hal.driver_setup = driver_setup;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.f_step_timer = gf_stream_vclk();
    hal.step_us_min = 1000000.0f / (float)gf_stream_rate();
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    driver_reset_chain = hal.driver_reset;
    hal.driver_reset = driverReset;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = glowforge_process_realtime;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.control.get_state = systemGetState;

    /* No spindle registered: the core adds a null spindle. The laser maps
     * to a laser-capable spindle in a later, scope-gated milestone. */

    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));

    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = eeprom_get_char;
    hal.nvs.put_byte = eeprom_put_char;
    hal.nvs.memcpy_to_nvs = memcpy_to_eeprom;
    hal.nvs.memcpy_from_nvs = memcpy_from_eeprom;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.irq_disable = irqDisable;
    hal.irq_enable = irqEnable;
    hal.get_elapsed_ticks = millis;
    hal.get_micros = micros;

    hal.driver_cap.amass_level = 3;
    /* Required for the hal to initialize properly (POST checks the bit
     * whether or not a pulse delay is configured). */
    hal.driver_cap.step_pulse_delay = On;

    return hal.version == 10;
}
