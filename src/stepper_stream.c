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
  process lifetime (kernel dead man's switch).

  Laser (bit 4 FIRE + bit-7 power bytes): the producer records power/
  fire transitions on the same tick grid as the steps (gf_stream_laser,
  called from the core's per-segment spindle update under the core
  lock); the shipper applies them as it emits - a power byte
  (0x80 | duty) ahead of the first tick byte the transition covers,
  then the FIRE bit OR'd into every tick byte while fire is on.
  Contract rules handled here: a power byte costs NO machine tick (the
  SDMA script processes the following byte in the same EPIT interrupt,
  so power bytes are extra stream bytes that leave the tick/due math
  untouched); consecutive power bytes would be dropped by the script
  (transitions are coalesced per tick, so a power byte always has a
  tick byte directly behind it); and every plain run start resets the
  hardware duty to ~100%, so a power byte leads every kernel run before
  any fire bit can occur. Arming policy lives in glowforge_laser.c: the
  kernel laser latch stays locked except inside an operator-armed job
  window, and while armed an underrun faults instead of the stop/run
  retry (a restarted run resets the duty - queued fire bits would
  replay at ~full power).

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

#include "fflog.h"
#include "stepper_stream.h"
#include "glowforge_homing.h"
#include "glowforge_io.h"
#include "driver.h"

#include "grbl/hal.h"

/* After the grbl headers: glibc's stat.h (via fcntl.h) defines an
 * st_mtime macro that would otherwise mangle the field of that name in
 * vfs.h. */
#include <fcntl.h>

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

/* GFSINK_RATE / GFSINK_DEPTH_MS bounds. The rate ceiling is the kernel
 * script's effective per-byte playback limit (~165 kHz); the depth floor
 * keeps at least one ship chunk in flight per shipper period, and the
 * ceiling (RING_SIZE / 2 bytes at the chosen rate) is applied at init. */
#define GFSINK_RATE_DEFAULT 28160
#define GFSINK_RATE_MIN 1000
#define GFSINK_RATE_MAX 165000
#define GFSINK_DEPTH_MS_DEFAULT 200
#define GFSINK_DEPTH_MS_MIN 20

/* Laser transition queue: must cover the transitions in flight between
 * the producer and the shipped cursor (~preload depth of stream time).
 * Grayscale engraving is the heavy case - a power change per pixel can
 * queue hundreds across a 200 ms window. */
#define LEV_BITS 12
#define LEV_N (1u << LEV_BITS)   /* 4096 */
#define LEV_MASK (LEV_N - 1)

struct laser_ev {
    uint64_t idx;             /* stream tick index the transition covers */
    uint8_t power;            /* raw 7-bit duty, 127 = full */
    uint8_t fire;             /* FIRE bit state from idx onward */
};

static struct {
    bool active;              /* device I/O enabled (GFSINK set) */
    bool suspended;           /* device handed to another process */
    const char *dev;          /* pulse device path (GFSINK) */
    int fd;
    int dump_fd;              /* GFSINK_DUMP: copy of the shipped stream */
    uint32_t rate;            /* machine tick = kernel step_freq */
    uint32_t depth;           /* preload/queue depth in bytes (ticks) */

    pthread_mutex_t lock;     /* guards everything below (lock order: core -> gf) */
    unsigned char ring[RING_SIZE];
    uint64_t produced;        /* stream index: next unwritten slot (producer) */
    uint64_t shipped;         /* stream index: next slot to ship (shipper) */
    uint64_t base;            /* stream index of the current wakeup's epoch */
    bool streaming;           /* motion in progress (wake_up .. go_idle) */
    bool kernel_running;      /* we believe a kernel run is active */
    bool run_pending;         /* bytes shipped, run refused (kernel busy):
                                 re-issue once the kernel is idle */
    bool clear_pending;       /* a reset stopped the kernel mid-motion: drop
                                 its unplayed residue once it is idle */
    bool hold_pending;        /* drop to hold currents once kernel idles */
    double hold_poll_at;      /* wall time of the next kernel-state poll */
    _Atomic bool failed;      /* unrecoverable; stop feeding (read lock-free
                                 by the entry points; written under gf.lock) */
    double ship_t0;           /* wall time when shipping started */
    uint64_t clamped;         /* events pushed forward (late vs cursor) */

    /* Laser transitions: produced on the tick grid, consumed by the
     * shipper as it emits. cur_* is the state already written into the
     * stream; power_sent guards the leading power byte per kernel run. */
    struct laser_ev lev[LEV_N];
    uint32_t lev_head;        /* consumer (shipper) */
    uint32_t lev_tail;        /* producer */
    uint8_t cur_power;
    bool cur_fire;
    bool power_sent;
    bool lev_overflow_warned;

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
    .dump_fd = -1,
    .period_latched = VTICKS_PER_BYTE,
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .wake_mx = PTHREAD_MUTEX_INITIALIZER,
    .wake_cv = PTHREAD_COND_INITIALIZER,
};

static _Atomic bool armed = false;
static _Atomic bool quit = false;
static _Atomic bool fault_flag = false;
static _Atomic bool laser_armed = false;

/* Serializes every cnc/laser_latch write in this process. The shipper's
 * run-start relight decision (sample laser_armed, then unlock) must be
 * atomic against a concurrent disarm's lock, or the relight can re-open
 * a latch the disarm just closed while FIRE bytes still sit in the
 * ring. Leaf lock: never held while taking gf.lock or the core lock. */
static pthread_mutex_t latch_mx = PTHREAD_MUTEX_INITIALIZER;

void gf_stream_laser_latch (bool lock)
{
    pthread_mutex_lock(&latch_mx);
    gfio_wr_attr("cnc/laser_latch", lock ? "1" : "0");
    pthread_mutex_unlock(&latch_mx);
}

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

/* Logging from the SCHED_FIFO shipper, or from under gf.lock: a raw
 * write(2) to stderr instead of stdio, so a real-time thread never
 * queues on the stdio FILE lock behind a normal-priority thread that is
 * mid-write to slow storage. */
static void rt_log (const char *msg)
{
    size_t len = strlen(msg);
    while(len) {
        ssize_t w = write(STDERR_FILENO, msg, len);
        if(w < 0 && errno == EINTR)
            continue;
        if(w <= 0)
            break;
        msg += w;
        len -= (size_t)w;
    }
}

/* Pulse-device write with the UAPI's backpressure semantics: -ENOMEM
 * (ring full) is "back off", not failure - retried on a short sleep for
 * a bounded time; EINTR is retried; a partial write is completed. The
 * bound is generous because the wall-clock pacing keeps the queue
 * shallow, so a ring that stays full is a kernel that has stopped
 * consuming and is caught by check_kernel_state(). Returns 0 or -1 with
 * errno set. */
#define PULSE_WRITE_BACKOFF_NS 2000000L   /* 2 ms */
#define PULSE_WRITE_BACKOFF_MAX 250       /* ~0.5 s of ring-full retries */

static int pulse_write (int fd, const unsigned char *buf, size_t len)
{
    int backoffs = 0;

    while(len) {
        ssize_t w = write(fd, buf, len);
        if(w > 0) {
            buf += w;
            len -= (size_t)w;
            continue;
        }
        if(w < 0 && errno == EINTR)
            continue;
        if(w < 0 && (errno == ENOMEM || errno == EAGAIN) &&
            ++backoffs <= PULSE_WRITE_BACKOFF_MAX) {
            sleep_ns(PULSE_WRITE_BACKOFF_NS);
            continue;
        }
        if(w == 0)
            errno = EIO;
        return -1;
    }
    return 0;
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
            rt_log("gfstream: ring overflow (producer runaway)\n");
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

/* Producer/laser: map a power/fire transition at the current virtual
 * time onto the pulse-byte grid. Idle-time calls are dropped: the core
 * re-asserts spindle power at the first segment of every laser block
 * (update_spindle_rpm is forced for laser blocks), and the kernel's
 * end-of-data backstop keeps the physical lines low between streams -
 * so nothing persists that the next stream does not restate. This also
 * keeps non-laser motion (jogs, homing) fire-free by construction. */
void gf_stream_laser (uint8_t power, bool fire)
{
    if(gf.failed)
        return;

    power &= 0x7f;

    pthread_mutex_lock(&gf.lock);
    if(gf.streaming) {
        uint64_t idx = gf.base + gf.vticks / VTICKS_PER_BYTE;
        if(idx < gf.shipped)
            idx = gf.shipped;

        uint32_t pending = gf.lev_tail - gf.lev_head;
        if(pending > 0 && gf.lev[(gf.lev_tail - 1) & LEV_MASK].idx == idx) {
            /* same machine tick: one transition per tick, last wins */
            struct laser_ev *e = &gf.lev[(gf.lev_tail - 1) & LEV_MASK];
            e->power = power;
            e->fire = fire;
        } else if(pending >= LEV_N) {
            /* queue full: merge into the newest slot so the final state
             * is exact (intermediate power steps are lost) */
            struct laser_ev *e = &gf.lev[(gf.lev_tail - 1) & LEV_MASK];
            e->power = power;
            e->fire = fire;
            if(!gf.lev_overflow_warned) {
                gf.lev_overflow_warned = true;
                rt_log("gfstream: laser transition queue overflow (merging)\n");
            }
        } else {
            gf.lev[gf.lev_tail & LEV_MASK] =
                (struct laser_ev){ .idx = idx, .power = power, .fire = fire };
            gf.lev_tail++;
        }
    }
    pthread_mutex_unlock(&gf.lock);
}

void gf_stream_laser_arm (bool state)
{
    laser_armed = state;
}

void gf_stream_wakeup (void)
{
    if(gf.failed)
        return;

    /* Factory run/idle scheme: full torque only while motion plays.
     * Playback starts a queue-depth behind, so the PIC settles first.
     * PIC writes ride SPI (milliseconds) - never under gf.lock. */
    if(gf.active)
        gfio_currents_run();

    pthread_mutex_lock(&gf.lock);
    if(!gf.streaming) {
        gf.streaming = true;
        gf.vticks = 0;
        gf.wall_epoch = wall_s();
        gf.hold_pending = false;
        if(!gf.kernel_running) {
            /* Fresh stream after a completed one: restart the stream-
             * relative index space (produced/shipped/due must share an
             * origin) and preload one depth of pad slots. Laser
             * transitions carry indices from the OLD space - stale ones
             * would clog the queue head forever; the previous stream
             * shipped out completely, so dropping them loses nothing. */
            gf.shipped = 0;
            gf.produced = gf.depth;
            gf.lev_head = gf.lev_tail;
            gf.cur_fire = false;
        } else {
            /* Continuation while the kernel still drains: the due/ship
             * cursor kept marching through the idle gap between cycles,
             * so production must resume AT the cursor's current wall
             * position, not at the old stream end - otherwise every new
             * event maps behind the cursor and clamps forward into step
             * bursts (a back-to-back return move loses steps with
             * thousands of clamps). The skipped slots ship as
             * zero pads: the machine really was idle for that time. */
            uint64_t due_now = (uint64_t)((wall_s() - gf.ship_t0) * gf.rate) + gf.depth;
            if(due_now > gf.produced) {
                gf.produced = due_now;
                /* The machine was idle across the gap, so the pads must
                 * ship dark whatever state the ended cycle left behind
                 * (belt and braces under the go_idle laser-off event). */
                gf.cur_fire = false;
            }
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
    if(gf.streaming) {
        /* The cycle's laser-off must be recorded HERE: the core issues
         * its spindle-off update after st_go_idle (and only for dynamic
         * blocks), when this stream no longer accepts transitions - so
         * without this event the FIRE bit would be inherited by the pad
         * bytes covering the idle gap to the next cycle: a stationary
         * full-power dwell burn at every planner starve. */
        if(gf.cur_fire || gf.lev_head != gf.lev_tail) {
            uint64_t idx = gf.produced > gf.shipped ? gf.produced : gf.shipped;
            uint32_t pending = gf.lev_tail - gf.lev_head;
            struct laser_ev *t = pending ?
                &gf.lev[(gf.lev_tail - 1) & LEV_MASK] : NULL;
            if(t && (t->idx >= idx || pending >= LEV_N))
                t->fire = false;    /* coalesce: the newest event wins */
            else {
                gf.lev[gf.lev_tail & LEV_MASK] = (struct laser_ev){
                    .idx = idx,
                    .power = t ? t->power : gf.cur_power,
                    .fire = false };
                gf.lev_tail++;
            }
        }
        gf.streaming = false;
    }
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

        if(n_calls) {
            pthread_mutex_lock(&gf.lock);
            uint64_t clamped_run = gf.clamped - clamped0;
            pthread_mutex_unlock(&gf.lock);
            fflog(LOG_DEBUG, "gfstream: run: %llu callbacks in %.3f s (%.1f us/call incl. pacing), "
                  "%llu pace sleeps, max behind %.1f ms, clamped %llu",
                  (unsigned long long)n_calls, wall_s() - t_run,
                  (wall_s() - t_run) * 1e6 / (double)n_calls,
                  (unsigned long long)slept, max_behind * 1e3,
                  (unsigned long long)clamped_run);
        }
    }

    return NULL;
}

/* --- shipper thread ------------------------------------------------------ */

/* Ship due bytes. Returns with gf.lock released. */
/* Start a kernel run on the bytes shipped so far. Returns 1 when the run
 * is playing (or a stalled one was recovered), 0 when the kernel is busy
 * (still draining a previous stream, or decelerating from a stop) - the
 * caller keeps the request pending and re-issues it once the kernel is
 * idle - 2 when the kernel is idle with nothing queued (everything was
 * already consumed), and -1 on a fault (flagged). Called without gf.lock. */
static int issue_run (void)
{
    char state[16] = "";
    int rc = 1;

    /* The kernel parks the FIRE line Hi-Z at every run end (its
     * laser-safe stop) and only a latch-unlock write restores SDMA
     * drive - the factory flow re-unlocks before every run. An
     * armed window spanning kernel runs must do the same, or the
     * next run's fire bits play into a tri-stated pin. The armed
     * check and the unlock are made atomic against a concurrent
     * disarm by latch_mx (see gf_stream_laser_latch). */
    pthread_mutex_lock(&latch_mx);
    bool relight = laser_armed;
    if(relight)
        gfio_wr_attr("cnc/laser_latch", "0");
    if(gfio_wr_attr("cnc/run", "1") != 0) {
        int err = errno;
        gfio_rd_attr("cnc/state", state, sizeof(state));
        if(strcmp(state, "underrun") == 0 && !laser_armed) {
            /* Late detection of a starve: ack, then retry once. The
             * counters no longer prove position, so the homing
             * anchor must not survive the retry. */
            rt_log("gfstream: kernel underrun; recovering - "
                   "position untrusted, re-home before an armed job\n");
            gfhome_invalidate();
            gfio_wr_attr("cnc/stop", "1");
            gfio_wr_attr("cnc/run", "1");
        } else if(strcmp(state, "underrun") == 0) {
            /* Laser armed: a restarted run resets the hardware duty,
             * so replaying the queued bytes could fire remaining
             * fire bits at ~full power. Ack the underrun and fail
             * safe instead; the alarm path relocks the latch. */
            rt_log("gfstream: kernel underrun while laser armed - failing safe\n");
            gfio_wr_attr("cnc/stop", "1");
            gf.failed = true;
            fault_flag = true;
            rc = -1;
        } else if(strcmp(state, "running") == 0) {
            /* Busy: the kernel is still playing the previous stream's
             * tail or decelerating from a stop. If it is still consuming
             * the ring, the bytes just written play as a continuation;
             * if it has already hit its end of data (or is stopping),
             * they wait in the ring for a run that nobody would issue -
             * and would replay at the start of the NEXT run instead.
             * Never assume: keep the request pending and re-issue it the
             * moment the kernel reads idle. */
            rc = 0;
        } else if(strcmp(state, "idle") == 0 && err == ENODATA) {
            rc = 2;                     /* nothing queued: already consumed */
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "gfstream: run refused (state=%s)\n", state);
            rt_log(msg);
            gf.failed = true;
            fault_flag = true;
            rc = -1;
        }
    }
    if(relight && !laser_armed)
        gfio_wr_attr("cnc/laser_latch", "1");  /* disarmed meanwhile - relock */
    pthread_mutex_unlock(&latch_mx);

    return rc;
}

/* Kernel-side follow-ups on the shipper's cadence: a run request the busy
 * kernel refused is re-issued once it is idle, and a mid-motion reset's
 * unplayed residue is dropped once the stop has played out. Both hold
 * shipping until resolved (a clear must not eat a new job's bytes).
 * Returns false when shipping must wait. */
static bool pending_pass (void)
{
    pthread_mutex_lock(&gf.lock);
    bool clear = gf.clear_pending, run = gf.run_pending, active = gf.active;
    pthread_mutex_unlock(&gf.lock);

    if(!active || (!clear && !run))
        return true;

    char state[16] = "";
    if(gfio_rd_attr("cnc/state", state, sizeof(state)) != 0)
        return !clear;
    if(strcmp(state, "running") == 0)
        return !clear;                  /* still playing: wait */

    if(clear) {
        /* Idle after the stop: whatever the ramp did not reach is stale
         * motion (and stale fire bits) that must never replay. */
        if(lseek(gf.fd, 1, SEEK_SET) < 0)
            rt_log("gfstream: could not clear the kernel residue after the reset\n");
        pthread_mutex_lock(&gf.lock);
        gf.clear_pending = false;
        gf.run_pending = false;
        gf.kernel_running = false;
        gf.power_sent = false;
        pthread_mutex_unlock(&gf.lock);
        return true;
    }

    /* Idle with our request pending: either the bytes we shipped were
     * consumed as a continuation (nothing left: run says no data) or they
     * were stranded and start now. */
    int rc = issue_run();
    pthread_mutex_lock(&gf.lock);
    if(rc == 1) {
        rt_log("gfstream: deferred run started (kernel was busy at the request)\n");
        gf.run_pending = false;
        gf.kernel_running = true;
    } else if(rc == 0) {
        /* raced back into running: keep waiting */
    } else {
        gf.run_pending = false;
        gf.kernel_running = false;
    }
    pthread_mutex_unlock(&gf.lock);
    return true;
}

static void ship_pass (void)
{
    static unsigned char chunk[SHIP_CHUNK];

    /* A deferred run or a residue clear first; a clear holds shipping. */
    if(!pending_pass())
        return;

    pthread_mutex_lock(&gf.lock);

    if(gf.failed || gf.suspended) {
        pthread_mutex_unlock(&gf.lock);
        return;
    }

    bool streaming = gf.streaming;
    uint64_t backlog = gf.produced > gf.shipped ? gf.produced - gf.shipped : 0;

    if(!streaming && backlog == 0) {
        bool drop_hold = false;
        /* stream finished: tell the kernel the next end-of-data is normal */
        if(gf.kernel_running) {
            if(gf.cur_fire) {
                /* Termination per the UAPI: a stream's last bytes must
                 * carry FIRE clear - the end-of-data backstop is the
                 * underrun safety net, never the normal path. If the
                 * final commanded byte fired, close the run with one
                 * dark pad tick. */
                unsigned char dark = 0x00;
                if(gf.dump_fd >= 0 && write(gf.dump_fd, &dark, 1) < 0) { /* debug copy only */ }
                if(gf.active && write(gf.fd, &dark, 1) < 0) { /* end-of-data still lands */ }
                gf.shipped++;
                gf.produced = gf.shipped;
                gf.cur_fire = false;
            }
            if(gf.active)
                gfio_wr_attr("cnc/streaming", "0");
            gf.kernel_running = false;   /* kernel drains the queue and idles */
            gf.power_sent = false;       /* the next run resets the duty */
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
                drop_hold = true;
                gf.hold_pending = false;
            } else
                gf.hold_poll_at = wall_s() + 0.1;
        }
        pthread_mutex_unlock(&gf.lock);
        if(drop_hold)
            gfio_currents_hold();   /* PIC-SPI: never under gf.lock */
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

        /* Up to 2 bytes per tick (power byte + tick byte). */
        while(n < SHIP_CHUNK - 1 && gf.shipped < due &&
               (streaming || gf.shipped < gf.produced)) {
            /* Apply the laser transitions this tick reaches, coalesced
             * to ONE power byte (the script drops back-to-back power
             * bytes; the tick byte below always follows it). A power
             * byte also MUST lead every kernel run (!power_sent): the
             * run start resets the hardware duty to ~100%, and only a
             * power byte ahead of the first fire bit closes that
             * full-power window. Power bytes cost no machine tick, so
             * insertions leave the due math untouched. */
            bool due_ev = false;
            uint8_t ev_power = gf.cur_power;
            while(gf.lev_head != gf.lev_tail &&
                   gf.lev[gf.lev_head & LEV_MASK].idx <= gf.shipped) {
                struct laser_ev *e = &gf.lev[gf.lev_head & LEV_MASK];
                ev_power = e->power;
                gf.cur_fire = e->fire;
                gf.lev_head++;
                due_ev = true;
            }
            if((due_ev && ev_power != gf.cur_power) || !gf.power_sent) {
                gf.cur_power = ev_power;
                gf.power_sent = true;
                chunk[n++] = 0x80 | gf.cur_power;
            }

            uint32_t slot = gf.shipped & RING_MASK;
            uint8_t b = gf.ring[slot];
            gf.ring[slot] = 0;           /* re-zero for the next lap */
            if(gf.cur_fire)
                b |= 0x10;
            chunk[n++] = b;
            gf.shipped++;
        }
        if(n == 0)
            break;
        if(gf.dump_fd >= 0 && write(gf.dump_fd, chunk, n) < 0) { /* debug copy only */ }
        if(gf.active && pulse_write(gf.fd, chunk, n) < 0) {
            char msg[96];
            snprintf(msg, sizeof(msg), "gfstream: pulse write failed: %s\n",
                     strerror(errno));
            rt_log(msg);
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
        int rc = issue_run();
        if(rc == 0) {
            /* Busy kernel: keep the request pending (pending_pass re-issues
             * it once the kernel is idle). kernel_running stays set: the
             * shipped bytes are the kernel's now, one way or the other. */
            pthread_mutex_lock(&gf.lock);
            gf.run_pending = true;
            pthread_mutex_unlock(&gf.lock);
        } else if(rc == 2) {
            /* Idle with nothing queued right after a write: the ring was
             * cleared under us; the stream restarts on the next ship. */
            pthread_mutex_lock(&gf.lock);
            gf.kernel_running = false;
            gf.power_sent = false;
            pthread_mutex_unlock(&gf.lock);
        }
    }
}

/* Mid-run fault detection at the shipper's cadence: the ring accepting
 * writes says nothing about playback, so a starve or a stepper-driver
 * fault would otherwise leave the sender planning and counting against
 * a motionless gantry until the NEXT run start. cnc/state is cheap to
 * read (free is the attr the UAPI forbids polling); read outside
 * gf.lock. driver.c turns the fault flag into disarm + anchor
 * invalidation + alarm on the protocol thread. */
static void check_kernel_state (void)
{
    bool running;

    pthread_mutex_lock(&gf.lock);
    running = gf.active && gf.kernel_running && !gf.failed && !gf.suspended;
    pthread_mutex_unlock(&gf.lock);
    if(!running)
        return;

    char state[16] = "";
    if(gfio_rd_attr("cnc/state", state, sizeof(state)) != 0)
        return;
    if(strcmp(state, "underrun") == 0 || strcmp(state, "fault") == 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "gfstream: kernel %s mid-run - alarming\n", state);
        rt_log(msg);
        pthread_mutex_lock(&gf.lock);
        gf.failed = true;
        pthread_mutex_unlock(&gf.lock);
        fault_flag = true;
    }
}

static void *shipper_thread (void *arg)
{
    (void)arg;

    /* Soft real time for the feeder per the UAPI recommendation; fall back
     * silently to normal scheduling without privileges. */
    struct sched_param sp = { .sched_priority = 10 };
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        rt_log("gfstream: SCHED_FIFO unavailable, using default scheduling\n");

    while(!quit) {
        ship_pass();
        check_kernel_state();
        sleep_ns(SHIP_PERIOD_NS);
    }

    return NULL;
}

/* --- reset / fault / lifecycle ------------------------------------------- */

void gf_stream_reset (void)
{
    /* Soft reset. Two cases:
     * - mid-motion (the stream is live): the core has declared position
     *   lost and gone idle without producing a decel. Drop the unshipped
     *   backlog and let the KERNEL decelerate (cnc/stop ramps down at
     *   ramp_rate), so nothing slams mechanically. The bytes already
     *   shipped that the ramp does not reach stay queued in the kernel
     *   ring and would replay at the start of the NEXT run - stale motion
     *   in an old direction; they are cleared once the kernel is idle
     *   (the position is lost anyway; see ship_pass).
     * - the stream had already ended (go_idle ran) and the kernel is only
     *   draining its completed queue: leave it. That queue is the tail of
     *   motion the core has counted; a stop here would strand it (to
     *   replay later) or lose it, and the shipper's end-of-stream logic
     *   finishes as it always does. */
    pthread_mutex_lock(&gf.lock);
    if(gf.streaming) {
        gf.produced = gf.shipped;
        gf.streaming = false;
        gf.lev_head = gf.lev_tail;   /* unshipped laser transitions die with the backlog */
        gf.cur_fire = false;
        gf.power_sent = false;
        if(gf.kernel_running) {
            if(gf.active) {
                gfio_wr_attr("cnc/stop", "1");
                gfio_wr_attr("cnc/streaming", "0");
            }
            gf.kernel_running = false;
            gf.run_pending = false;
            gf.clear_pending = true;
            gf.hold_pending = true;
            gf.hold_poll_at = wall_s() + 0.2;
        }
    }
    /* else: the completed stream's tail keeps shipping and draining. */
    pthread_mutex_unlock(&gf.lock);
}

bool gf_stream_fault_take (void)
{
    return atomic_exchange(&fault_flag, false);
}

void gf_stream_clear_position (void)
{
    pthread_mutex_lock(&gf.lock);
    if(gf.active && gf.fd >= 0)
        lseek(gf.fd, 2, SEEK_SET);   /* clear the kernel position counters */
    pthread_mutex_unlock(&gf.lock);
}

bool gf_stream_suspend (void)
{
    if(!gf.active)
        return true;

    bool idle, drop_hold = false;

    pthread_mutex_lock(&gf.lock);
    /* A pending residue clear must land before the device changes hands:
     * stale bytes would replay in the other holder's first run. */
    idle = !gf.streaming && !gf.kernel_running && !gf.clear_pending &&
            gf.produced == gf.shipped;
    if(idle) {
        /* The kernel may still be playing out its queue (the decel tail
         * lives there); closing a flock'd fd mid-program is an emergency
         * stop, so hand over only from a truly idle device. */
        char state[16] = "";
        idle = gfio_rd_attr("cnc/state", state, sizeof(state)) == 0 &&
                strcmp(state, "idle") == 0;
    }
    if(idle) {
        if(gf.hold_pending) {
            drop_hold = true;
            gf.hold_pending = false;
        }
        gf.suspended = true;
        /* Under the broker the device stays open across the handover
         * (the homing runner inherits the same description through its
         * environment); the producer is gated by the suspended flag.
         * Standalone, the close is the handover. */
        if(!gfio_pulse_inherited()) {
            close(gf.fd);
            gf.fd = -1;
        }
    }
    pthread_mutex_unlock(&gf.lock);

    if(drop_hold)
        gfio_currents_hold();   /* PIC-SPI: never under gf.lock */

    return idle;
}

/* The 40 V motor rail tolerates a clean power-up but not a fast off/on
 * bounce: re-enabling within ~tens of ms of the previous drop can leave
 * the supply folded back, where the SDMA stream plays and counts
 * normally while the motors produce no torque or stall mid-move. Every
 * takeover of the pulse device therefore starts with a deliberate off
 * period so the rail always powers up from a settled state.
 * Conf key: rail_settle_s (seconds, 0 disables). */
#define RAIL_SETTLE_S_DEFAULT 2.5f

static void rail_settle (void)
{
    float s = gfio_conf_read_float("rail_settle_s", RAIL_SETTLE_S_DEFAULT);
    if(s <= 0.0f)
        return;
    gfio_wr_attr("cnc/disable", "1");
    fflog(LOG_INFO, "gfstream: settling motor rail for %.1f s", s);
    for(double end = wall_s() + (double)s; wall_s() < end; )
        sleep_ns(50000000);
}

bool gf_stream_resume (void)
{
    if(!gf.active)
        return true;

    int fd;
    if(gfio_pulse_inherited()) {
        /* Broker: we kept the fd across the handover and the device
         * never closed - the rail never dropped, so there is nothing
         * to settle. */
        fd = gf.fd;
    } else {
        /* The previous owner has exited but its fd close can lag; take
         * the lock non-blocking with a short retry so the protocol
         * thread never hangs here. */
        fd = -1;
        for(int tries = 0; fd < 0 && tries < 50; tries++) {
            if((fd = gfio_open_pulse_dev_nb(gf.dev)) < 0)
                sleep_ns(100000000);   /* 100 ms */
        }
        if(fd < 0) {
            fflog(LOG_ERR, "gfstream: cannot reacquire %s", gf.dev);
            return false;
        }
    }

    /* All of the following runs outside gf.lock: gf.suspended keeps the
     * shipper out until the final state swap, and rail_settle alone is
     * seconds - a SCHED_FIFO thread must never wait on that. */

    /* Standalone, the exiting session dropped the 40 V rail moments ago
     * and our open bounced it back on; give the supply a real off
     * period first. */
    if(!gfio_pulse_inherited())
        rail_settle();

    /* The homing session reconfigured the machine; re-apply the full
     * analog config and stream state exactly as at init. */
    gfio_analog_config();
    char val[16];
    snprintf(val, sizeof(val), "%u", gf.rate);
    bool ok = gfio_wr_attr("cnc/step_freq", val) == 0;
    lseek(fd, 1, SEEK_SET);           /* clear pulse data + byte counters */
    gfio_wr_attr("cnc/stop", "1");    /* ack a stale underrun if latched */
    gfio_wr_attr("cnc/enable", "1");  /* steppers on (idle) */

    pthread_mutex_lock(&gf.lock);
    gf.fd = fd;
    gf.produced = gf.shipped = 0;
    gf.streaming = false;
    gf.kernel_running = false;
    gf.hold_pending = false;
    gf.failed = false;
    gf.suspended = false;
    gf.lev_head = gf.lev_tail;
    gf.cur_power = 0;
    gf.cur_fire = false;
    gf.power_sent = false;
    pthread_mutex_unlock(&gf.lock);

    if(!ok)
        fflog(LOG_ERR, "gfstream: cannot restore step_freq");

    return ok;
}

void gf_stream_init (void)
{
    const char *dev = getenv("GFSINK"), *opt;
    char val[16];

    /* gf.lock is contended by the SCHED_FIFO shipper against two
     * normal-priority threads on a uniprocessor: priority inheritance
     * bounds the inversion when a preempted holder blocks the shipper.
     * Rebuilt here, before any user of the lock runs. */
    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_destroy(&gf.lock);
    pthread_mutex_init(&gf.lock, &ma);
    pthread_mutexattr_destroy(&ma);

    /* Hardware writes stay gated off for null-sink instances. */
    gfio_set_hw(dev != NULL && *dev != '\0');

    /* Machine tick and queue depth. Out-of-range values fall back to the
     * defaults with a warning rather than being applied: a zero rate is
     * a division by zero in the pacing math, a rate above the kernel's
     * effective playback ceiling underruns by construction, and a depth
     * the stream ring cannot hold faults on the first ship. */
    gf.rate = GFSINK_RATE_DEFAULT;
    if((opt = getenv("GFSINK_RATE")) && *opt) {
        long v = strtol(opt, NULL, 10);
        if(v >= GFSINK_RATE_MIN && v <= GFSINK_RATE_MAX)
            gf.rate = (uint32_t)v;
        else
            fflog(LOG_WARNING, "gfstream: GFSINK_RATE '%s' out of range (%u-%u); using %u",
                  opt, GFSINK_RATE_MIN, GFSINK_RATE_MAX, GFSINK_RATE_DEFAULT);
    }
    uint32_t depth_ms = GFSINK_DEPTH_MS_DEFAULT;
    if((opt = getenv("GFSINK_DEPTH_MS")) && *opt) {
        long v = strtol(opt, NULL, 10);
        if(v >= GFSINK_DEPTH_MS_MIN && (uint64_t)gf.rate * (uint64_t)v / 1000 <= RING_SIZE / 2)
            depth_ms = (uint32_t)v;
        else
            fflog(LOG_WARNING, "gfstream: GFSINK_DEPTH_MS '%s' out of range at %u Hz; using %u",
                  opt, gf.rate, GFSINK_DEPTH_MS_DEFAULT);
    }
    gf.depth = (uint32_t)((uint64_t)gf.rate * depth_ms / 1000);

    /* Bench/debug: mirror every shipped byte to a file for offline
     * stream inspection (works in null-sink mode too). */
    if((opt = getenv("GFSINK_DUMP")) && *opt)
        gf.dump_fd = open(opt, O_WRONLY | O_CREAT | O_TRUNC, 0644);

    if(dev != NULL && *dev != '\0') {

        gf.dev = dev;
        if((gf.fd = gfio_open_pulse_dev(dev)) < 0) {
            fflog(LOG_ERR, "gfstream: cannot open %s", dev);
            exit(1);
        }

        /* Standalone, this open may have re-powered the 40 V rail
         * moments after the previous holder dropped it; settle it
         * before configuring. Under the broker the device (and the
         * rail's state) is continuous across our restarts. */
        if(!gfio_pulse_inherited())
            rail_settle();
        else
            fflog(LOG_INFO, "gfstream: pulse device inherited from the broker");

        /* Laser latch locked at init (glowforge_laser.c unlocks it only
         * inside an operator-armed job window); then the full factory
         * analog config (modes, decay, motor lock, hold currents). */
        gfio_analog_config();
        snprintf(val, sizeof(val), "%u", gf.rate);
        if(gfio_wr_attr("cnc/step_freq", val) != 0) {
            fflog(LOG_ERR, "gfstream: cannot set step_freq");
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
        fflog(LOG_ERR, "gfstream: cannot start stream threads");
        exit(1);
    }
    gf.threads_started = true;

    atexit(gf_stream_shutdown);
    fflog(LOG_INFO, "gfstream: %s, %u Hz machine tick, %u ms depth",
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

    if(gf.active && !gf.suspended) {
        gfio_wr_attr("cnc/streaming", "0");
        gfio_wr_attr("cnc/halt", "1");
    }
    /* Under the broker this close is not the final close of the pulse
     * device, so the kernel's close-relock does not fire; relock
     * explicitly (a no-op when already locked). */
    if(gf.active)
        gfio_wr_attr("cnc/laser_latch", "1");
    if(gf.clamped)
        fflog(LOG_WARNING, "gfstream: %llu late events clamped",
              (unsigned long long)gf.clamped);
    if(gf.fd >= 0) {
        close(gf.fd);
        gf.fd = -1;
    }
    if(gf.dump_fd >= 0) {
        close(gf.dump_fd);
        gf.dump_fd = -1;
    }
}
