/*
  stepper_stream.c - pulse-stream stepper engine for the Glowforge board

  Part of grblHAL-glowforge. This module is the step "timer": it turns
  grblHAL stepper events directly into the kernel pulse-byte stream.

  Contract (kernel-module-glowforge/UAPI.md, "Pulse-stream feeder
  contract"): one byte per machine tick (kernel step_freq); velocity is
  step density; byte layout bit0 X_STEP, bit1 X_DIR (set = -X), bit2
  Y_STEP, bit3 Y_DIR (set = +Y), bit5 Z_STEP, bit6 Z_DIR (set = +Z, lens
  up); streaming=1 while live-feeding (underrun is fault-like), cleared
  before the terminal end-of-data; the fd stays open and flock'd for the
  process lifetime (kernel dead man's switch). MOTION ONLY: bit 4 (laser)
  is never set and the laser latch is forced locked at init.

  Threading model (three actors):

  - The grbl protocol thread runs the planner and calls the hal.stepper
    entry points below via driver.c (always under the recursive core
    lock).
  - The PRODUCER thread: while armed it runs
    hal.stepper.interrupt_callback() under the core
    lock, advances a virtual clock (hal.f_step_timer = 1000 x machine
    tick) by the latched cycles_per_tick, and maps each pulse event onto
    the byte grid (exact /1000). It is wall-clock paced to stay within
    [wall, wall + ~2 ms]: the core's internal segment lookahead is only
    ~40-90 ms and is refilled from the protocol thread, so production
    must not run ahead of real time (the queue depth is supplied by pad
    PRELOAD, not by producing early).
  - The SHIPPER thread (SCHED_FIFO when permitted) writes due bytes to
    /dev/glowforge every ~10 ms (due = wall-elapsed x rate + depth, per
    the UAPI pacing rule) and owns the kernel state machine: run start,
    underrun ack/retry, streaming=1/0, and the PIC run/hold current
    switching (hold applied only after the kernel has drained its queue
    and idled - the decel tail lives there).

  Lock order is strictly core -> gf. The shipper takes only gf.lock and
  NEVER calls core APIs; stream faults are surfaced through an atomic
  flag polled by the protocol thread's realtime hook.

  Without GFSINK the module runs in null-sink mode: producer, ring and
  shipper all operate identically but no device/sysfs I/O happens -
  used for hardware-less protocol/motion verification.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "stepper_stream.h"
#include "glowforge_io.h"
#include "driver.h"

#include "grbl/hal.h"

/* Stream ring: 1 << 17 = 131072 machine ticks of headroom (4.6 s @ 28160).
 * With the producer wall-paced the lead over the shipped cursor stays near
 * the preload depth; the guard below is a belt-and-braces failsafe. */
#define RING_BITS 17
#define RING_SIZE (1u << RING_BITS)
#define RING_MASK (RING_SIZE - 1)

/* Ship chunk: bounds write frequency (9.1 ms of stream @ 28160). Must be
 * well under the preload depth or the kernel starves between ships. */
#define SHIP_CHUNK 256

/* Virtual step-clock ticks per pulse byte: hal.f_step_timer is defined as
 * 1000 x the machine tick, so this is exact. */
#define VTICKS_PER_BYTE 1000

/* Producer pacing: run interrupt callbacks while virtual time is less than
 * wall + SLACK; sleep in PACE_SLICE steps while ahead. */
#define PACE_SLACK_S 0.002
#define PACE_SLICE_NS 1000000   /* 1 ms */

/* Shipper cadence. */
#define SHIP_PERIOD_NS 10000000 /* 10 ms */

static struct {
    bool active;              /* device I/O enabled (GFSINK set) */
    int fd;
    uint32_t rate;            /* machine tick = kernel step_freq */
    uint32_t depth;           /* preload/queue depth in bytes (ticks) */

    pthread_mutex_t lock;     /* guards everything below (lock order: core -> gf) */
    unsigned char ring[RING_SIZE];
    uint64_t produced;        /* stream index: next unwritten slot (producer) */
    uint64_t shipped;         /* stream index: next slot to ship (shipper) */
    uint64_t base;            /* stream index of the current wakeup's epoch */
    bool streaming;           /* motion in progress (wake_up .. go_idle) */
    bool kernel_running;      /* we believe a kernel run is active */
    bool hold_pending;        /* drop to hold currents once kernel idles */
    double hold_poll_at;      /* wall time of the next kernel-state poll */
    bool failed;              /* unrecoverable; stop feeding */
    double ship_t0;           /* wall time when shipping started */
    uint64_t clamped;         /* events pushed forward (late vs cursor) */

    /* Producer state: vticks/period written only under the core lock. */
    uint64_t vticks;          /* virtual step-clock time since wakeup epoch */
    double wall_epoch;        /* wall time of the wakeup epoch */
    uint32_t period_latched;  /* current cycles_per_tick; persists across jobs
                                 (the core's own cache does too - wake_up must
                                 never reset this, see stepper.c cache reset) */

    pthread_mutex_t wake_mx;
    pthread_cond_t wake_cv;
    pthread_t producer_tid;
    pthread_t shipper_tid;
    bool threads_started;
} gf = {
    .fd = -1,
    .period_latched = VTICKS_PER_BYTE,
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake_mx = PTHREAD_MUTEX_INITIALIZER,
    .wake_cv = PTHREAD_COND_INITIALIZER,
};

static _Atomic bool armed = false;
static _Atomic bool quit = false;
static _Atomic bool fault_flag = false;

static double wall_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static void sleep_ns (long ns)
{
    struct timespec ts = { .tv_sec = 0, .tv_nsec = ns };
    nanosleep(&ts, NULL);
}

uint32_t gf_stream_vclk (void)
{
    return gf.rate * VTICKS_PER_BYTE;
}

uint32_t gf_stream_rate (void)
{
    return gf.rate;
}

/* --- hal.stepper entry points (called under the core lock) --------------- */

void gf_stream_cycles_per_tick (uint32_t cycles)
{
    gf.period_latched = cycles == 0 ? 1 : cycles;
}

/* Producer/pulse: map a step event at the current virtual time onto the
 * pulse-byte grid. grblHAL bits: bit0=X bit1=Y bit2=Z, dir bit set =
 * negative direction. Pulse byte: X_DIR set = -X (direct), Y_DIR set = +Y
 * (inverted), Z_DIR set = +Z/up (inverted). */
void gf_stream_pulse (uint8_t step_bits, uint8_t dir_bits)
{
    if(gf.failed)
        return;

    unsigned char b = 0;
    if(step_bits & 0x01)
        b |= 0x01 | ((dir_bits & 0x01) ? 0x02 : 0);
    if(step_bits & 0x02)
        b |= 0x04 | ((dir_bits & 0x02) ? 0 : 0x08);
    if(step_bits & 0x04)
        b |= 0x20 | ((dir_bits & 0x04) ? 0 : 0x40);
    if(b == 0)
        return;

    pthread_mutex_lock(&gf.lock);
    if(gf.streaming) {
        uint64_t idx = gf.base + gf.vticks / VTICKS_PER_BYTE;
        if(idx < gf.shipped) {          /* produced late vs wall clock: push forward */
            idx = gf.shipped;
            gf.clamped++;
        }
        if(idx < gf.produced)           /* same machine tick as a previous event */
            idx = gf.produced;
        if(idx - gf.shipped >= RING_SIZE - 1) {
            fprintf(stderr, "gfstream: ring overflow (producer runaway)\n");
            gf.failed = true;
            fault_flag = true;
        } else {
            /* slots between produced and idx stay 0x00 (zeroed after ship) */
            gf.ring[idx & RING_MASK] = b;
            gf.produced = idx + 1;
        }
    }
    pthread_mutex_unlock(&gf.lock);
}

void gf_stream_wakeup (void)
{
    if(gf.failed)
        return;

    pthread_mutex_lock(&gf.lock);
    if(!gf.streaming) {
        gf.streaming = true;
        gf.vticks = 0;
        gf.wall_epoch = wall_s();
        /* Factory run/idle scheme: full torque only while motion plays.
         * Playback starts a queue-depth behind, so the PIC settles first. */
        gf.hold_pending = false;
        if(gf.active)
            gfio_currents_run();
        if(!gf.kernel_running) {
            /* Fresh stream after a completed one: restart the stream-
             * relative index space (produced/shipped/due must share an
             * origin) and preload one depth of pad slots. */
            gf.shipped = 0;
            gf.produced = gf.depth;
        } else {
            /* Continuation while the kernel still drains: the due/ship
             * cursor kept marching through the idle gap between cycles,
             * so production must resume AT the cursor's current wall
             * position, not at the old stream end - otherwise every new
             * event maps behind the cursor and clamps forward into step
             * bursts (observed live: a back-to-back return move lost
             * steps with thousands of clamps). The skipped slots ship as
             * zero pads: the machine really was idle for that time. */
            uint64_t due_now = (uint64_t)((wall_s() - gf.ship_t0) * gf.rate) + gf.depth;
            if(due_now > gf.produced)
                gf.produced = due_now;
        }
        gf.base = gf.produced;
        if(gf.active)
            gfio_wr_attr("cnc/streaming", "1");
    }
    pthread_mutex_unlock(&gf.lock);

    /* Arm the producer. NOTE: st_wake_up calls go_idle(true) first, so a
     * back-to-back job disarms/re-arms; the shipper may slip into that gap
     * and treat the stream as finished - benign (extra streaming toggle +
     * one preload of added latency), see the design notes. */
    pthread_mutex_lock(&gf.wake_mx);
    armed = true;
    pthread_cond_signal(&gf.wake_cv);
    pthread_mutex_unlock(&gf.wake_mx);
}

void gf_stream_go_idle (void)
{
    armed = false;

    pthread_mutex_lock(&gf.lock);
    gf.streaming = false;
    pthread_mutex_unlock(&gf.lock);
}

/* --- producer thread ----------------------------------------------------- */

static void *producer_thread (void *arg)
{
    (void)arg;

    for(;;) {
        pthread_mutex_lock(&gf.wake_mx);
        while(!armed && !quit)
            pthread_cond_wait(&gf.wake_cv, &gf.wake_mx);
        pthread_mutex_unlock(&gf.wake_mx);

        if(quit)
            break;

        uint64_t n_calls = 0, slept = 0;
        double t_run = wall_s(), max_behind = 0.0;

        pthread_mutex_lock(&gf.lock);
        uint64_t clamped0 = gf.clamped;
        pthread_mutex_unlock(&gf.lock);

        while(armed && !quit) {

            gf_core_lock();
            if(!armed) {
                gf_core_unlock();
                break;
            }
            /* One virtual timer fire. The callback may: emit a pulse (we
             * map it at the CURRENT vticks), change the period (applies to
             * the interval after this fire, matching hardware timer reload
             * semantics), or go idle (buffer drained / job done). */
            hal.stepper.interrupt_callback();
            uint64_t period = gf.period_latched;
            gf.vticks += period;
            double vtime = (double)gf.vticks / (double)gf_stream_vclk();
            double epoch = gf.wall_epoch;
            gf_core_unlock();

            n_calls++;
            double behind = (wall_s() - epoch) - vtime;
            if(behind > max_behind)
                max_behind = behind;

            /* Pace: keep virtual time within [wall, wall + slack] of the
             * wakeup epoch. Sleeps are sliced so disarm is noticed fast
             * even inside a multi-second G4 tick. Yield when running flat
             * out so the protocol thread gets lock windows (single core). */
            if(vtime > (wall_s() - epoch) + PACE_SLACK_S) {
                do {
                    slept++;
                    sleep_ns(PACE_SLICE_NS);
                } while(armed && !quit && vtime > (wall_s() - epoch) + PACE_SLACK_S);
            } else
                sched_yield();
        }

        if(n_calls)
            fprintf(stderr, "gfstream: run: %llu callbacks in %.3f s (%.1f us/call incl. pacing), "
                    "%llu pace sleeps, max behind %.1f ms, clamped %llu\n",
                    (unsigned long long)n_calls, wall_s() - t_run,
                    (wall_s() - t_run) * 1e6 / (double)n_calls,
                    (unsigned long long)slept, max_behind * 1e3,
                    (unsigned long long)(gf.clamped - clamped0));
    }

    return NULL;
}

/* --- shipper thread ------------------------------------------------------ */

/* Ship due bytes. Returns with gf.lock released. */
static void ship_pass (void)
{
    static unsigned char chunk[SHIP_CHUNK];

    pthread_mutex_lock(&gf.lock);

    if(gf.failed) {
        pthread_mutex_unlock(&gf.lock);
        return;
    }

    bool streaming = gf.streaming;
    uint64_t backlog = gf.produced > gf.shipped ? gf.produced - gf.shipped : 0;

    if(!streaming && backlog == 0) {
        /* stream finished: tell the kernel the next end-of-data is normal */
        if(gf.kernel_running) {
            if(gf.active)
                gfio_wr_attr("cnc/streaming", "0");
            gf.kernel_running = false;   /* kernel drains the queue and idles */
            gf.hold_pending = true;
            gf.hold_poll_at = wall_s() + 0.1;
        }
        /* Motors keep run current until the kernel has actually played out
         * its queue (the decel tail lives there); then drop to hold. */
        if(gf.hold_pending && wall_s() >= gf.hold_poll_at) {
            char state[16] = "";
            if(!gf.active) {
                gf.hold_pending = false;
            } else if(gfio_rd_attr("cnc/state", state, sizeof(state)) == 0 &&
                       strcmp(state, "idle") == 0) {
                gfio_currents_hold();
                gf.hold_pending = false;
            } else
                gf.hold_poll_at = wall_s() + 0.1;
        }
        pthread_mutex_unlock(&gf.lock);
        return;
    }

    /* Wall-clock due index: keep the kernel exactly depth ahead of real
     * time. Until the first ship, ship_t0 anchors the stream to the wall. */
    if(gf.shipped == 0 || !gf.kernel_running) {
        if(backlog < (streaming ? gf.depth : 1)) {  /* wait for the preload */
            pthread_mutex_unlock(&gf.lock);
            return;
        }
        gf.ship_t0 = wall_s();
    }

    uint64_t due = (uint64_t)((wall_s() - gf.ship_t0) * gf.rate) + gf.depth;
    bool start_run = false;

    while(gf.shipped < due && (streaming || gf.shipped < gf.produced)) {
        uint32_t n = 0;
        while(n < SHIP_CHUNK && gf.shipped < due &&
               (streaming || gf.shipped < gf.produced)) {
            uint32_t slot = gf.shipped & RING_MASK;
            chunk[n++] = gf.ring[slot];
            gf.ring[slot] = 0;           /* re-zero for the next lap */
            gf.shipped++;
        }
        if(n == 0)
            break;
        if(gf.active && write(gf.fd, chunk, n) < 0) {
            fprintf(stderr, "gfstream: pulse write failed: %s\n", strerror(errno));
            gf.failed = true;
            fault_flag = true;
            break;
        }
        if(!gf.kernel_running) {
            start_run = true;
            gf.kernel_running = true;
        }
    }

    pthread_mutex_unlock(&gf.lock);

    if(start_run && gf.active && !gf.failed) {
        char state[16] = "";
        if(gfio_wr_attr("cnc/run", "1") != 0) {
            gfio_rd_attr("cnc/state", state, sizeof(state));
            if(strcmp(state, "underrun") == 0) {
                /* late detection of a starve: ack, then retry once */
                fprintf(stderr, "gfstream: kernel underrun; recovering\n");
                gfio_wr_attr("cnc/stop", "1");
                gfio_wr_attr("cnc/run", "1");
            } else if(strcmp(state, "running") != 0) {
                fprintf(stderr, "gfstream: run refused (state=%s)\n", state);
                gf.failed = true;
                fault_flag = true;
            }
        }
    }
}

static void *shipper_thread (void *arg)
{
    (void)arg;

    /* Soft real time for the feeder per the UAPI recommendation; fall back
     * silently to normal scheduling without privileges. */
    struct sched_param sp = { .sched_priority = 10 };
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "gfstream: SCHED_FIFO unavailable, using default scheduling\n");

    while(!quit) {
        ship_pass();
        sleep_ns(SHIP_PERIOD_NS);
    }

    return NULL;
}

/* --- reset / fault / lifecycle ------------------------------------------- */

void gf_stream_reset (void)
{
    /* Soft reset mid-motion: the core has already declared position lost
     * and gone idle without producing a decel. Drop the unshipped backlog
     * and let the KERNEL decelerate (cnc/stop ramps down at ramp_rate), so
     * nothing slams mechanically. */
    pthread_mutex_lock(&gf.lock);
    gf.produced = gf.shipped;
    gf.streaming = false;
    if(gf.kernel_running) {
        if(gf.active) {
            gfio_wr_attr("cnc/stop", "1");
            gfio_wr_attr("cnc/streaming", "0");
        }
        gf.kernel_running = false;
        gf.hold_pending = true;
        gf.hold_poll_at = wall_s() + 0.2;
    }
    pthread_mutex_unlock(&gf.lock);
}

bool gf_stream_fault_take (void)
{
    return atomic_exchange(&fault_flag, false);
}

void gf_stream_init (void)
{
    const char *dev = getenv("GFSINK"), *opt;
    char val[16];

    gf.rate = (opt = getenv("GFSINK_RATE")) ? (uint32_t)atoi(opt) : 28160;
    uint32_t depth_ms = (opt = getenv("GFSINK_DEPTH_MS")) ? (uint32_t)atoi(opt) : 200;
    gf.depth = (uint32_t)((uint64_t)gf.rate * depth_ms / 1000);

    if(dev != NULL && *dev != '\0') {

        if((gf.fd = gfio_open_pulse_dev(dev)) < 0) {
            fprintf(stderr, "gfstream: cannot open %s\n", dev);
            exit(1);
        }

        /* Motion only: laser locked out at the driver, whatever else
         * happens; then the full factory analog config (modes, decay,
         * motor lock, hold currents). */
        gfio_analog_config();
        snprintf(val, sizeof(val), "%u", gf.rate);
        if(gfio_wr_attr("cnc/step_freq", val) != 0) {
            fprintf(stderr, "gfstream: cannot set step_freq\n");
            exit(1);
        }
        /* Fresh stream state. */
        lseek(gf.fd, 1, SEEK_SET);        /* clear pulse data + byte counters */
        gfio_wr_attr("cnc/stop", "1");    /* ack a stale underrun if latched */
        gfio_wr_attr("cnc/enable", "1");  /* steppers on (idle) */

        gf.active = true;
    }

    if(pthread_create(&gf.producer_tid, NULL, producer_thread, NULL) != 0 ||
        pthread_create(&gf.shipper_tid, NULL, shipper_thread, NULL) != 0) {
        fprintf(stderr, "gfstream: cannot start stream threads\n");
        exit(1);
    }
    gf.threads_started = true;

    atexit(gf_stream_shutdown);
    fprintf(stderr, "gfstream: %s, %u Hz machine tick, %u ms depth\n",
            gf.active ? dev : "null-sink (no GFSINK)", gf.rate, depth_ms);
}

void gf_stream_shutdown (void)
{
    static bool done = false;

    if(done || !gf.threads_started)
        return;
    done = true;

    quit = true;
    armed = false;
    pthread_mutex_lock(&gf.wake_mx);
    pthread_cond_signal(&gf.wake_cv);
    pthread_mutex_unlock(&gf.wake_mx);
    pthread_join(gf.producer_tid, NULL);
    pthread_join(gf.shipper_tid, NULL);

    if(gf.active) {
        gfio_wr_attr("cnc/streaming", "0");
        gfio_wr_attr("cnc/halt", "1");
    }
    if(gf.clamped)
        fprintf(stderr, "gfstream: %llu late events clamped\n",
                (unsigned long long)gf.clamped);
    if(gf.fd >= 0) {
        close(gf.fd);
        gf.fd = -1;
    }
}
