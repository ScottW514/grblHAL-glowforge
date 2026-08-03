/*
  glowforge_homing.c - accelerometer bump-detect homing signals

  Part of grblHAL-glowforge. The factory machine has no X/Y home
  switches. Homing instead creeps toward a rail and detects the contact
  jolt on the head's LIS2HH12 accelerometer (i2c-3 addr 0x1e), reported
  to the core as a virtual limit switch in limit_signals_t.min - the
  core's standard seek/pull-off/locate homing cycle runs unmodified.

  Why this is accurate: after the jolt triggers, the core kills step
  generation, but bytes already queued in the kernel stream still play
  out - those steps skip harmlessly against the rail, leaving the head
  pressed at the physical reference. The following pull-off is a clean
  move away from that reference, so the homed origin is rail + pull-off
  regardless of any position-counter drift from the skipped steps.

  Sensor access is direct I2C: the kernel st_accel driver's one-shot
  sysfs reads are ~6 Hz (it power-cycles the chip per read) and this
  kernel carries no IIO triggers, so while armed the monitor unbinds
  st-accel from the chip, programs 800 Hz ODR, and burst-reads OUT_X..Z
  at ~1 kHz (rebinding on disarm). The i2c core serializes bus access
  against the kernel's other clients on the head bus.

  Detection is per approach phase: the core's on_homing_rate_set event
  marks each Seek/Locate/Pulloff phase, and every approach runs a fresh
  ramp-skip + baseline-learn + detect session (the locate creep is much
  quieter than the seek, so it earns its own tighter threshold), while
  pull-off phases suspend detection entirely - the reversal and stop
  jerks of a pull-off would otherwise read as contact against a
  quiet-baseline threshold, failing the pull-off completion check.
  Trigger condition: summed 3-axis deviation from an EMA gravity tracker
  exceeding max(mean + k*sigma, floor) for two consecutive samples.
  Bench-measured contact jolts run 20-40x the creep baseline and cross
  within ~4 ms. An approach whose learned "baseline" is itself at
  grinding level means it started pressed against the rail and triggers
  immediately.

  If the sensor fails while armed the monitor injects EXEC_RESET so the
  homing cycle aborts safely instead of grinding to the over-travel
  alarm.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

/* grbl headers first: glibc's stat.h (via fcntl.h) defines an st_mtime
 * macro that would otherwise mangle the field of that name in vfs.h */
#include "driver.h"
#include "glowforge_homing.h"

#include "grbl/system.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define I2C_SLAVE       0x0703
#define HOME_I2C_BUS    "/dev/i2c-3"
#define HOME_I2C_ADDR   0x1e
#define REG_CTRL1       0x20
#define CTRL1_800HZ_XYZ 0x6F        /* 800 Hz ODR, BDU, XYZ enable */
#define REG_OUT_X_L     0x28
#define ST_ACCEL_UNBIND "/sys/bus/i2c/drivers/st-accel-i2c/unbind"
#define ST_ACCEL_BIND   "/sys/bus/i2c/drivers/st-accel-i2c/bind"
#define ST_ACCEL_DEV    "3-001e"

#define SAMPLE_US       900         /* ~1 kHz poll */
#define RAMP_MS         150         /* ignore approach accel transients */
#define LEARN_MS        350         /* moving-baseline learning window */
#define CONTACT_BASELINE 10000.0f   /* learned "baseline" this high means
                                     * the approach started pressed against
                                     * the rail and is already grinding
                                     * (normal creep is ~3-5k, grinding
                                     * 20-40k): treat as contact */
#define CONFIRM_SAMPLES 2
#define TRIG_HOLD_MS    300         /* < pull-off duration, > core poll */
#define MAX_I2C_ERRORS  5
#define EMA_ALPHA       0.05f
#define K_SIGMA_DEFAULT 8.0f
#define FLOOR_DEFAULT   3000.0f     /* counts of summed 3-axis deviation */

static struct {
    pthread_t       tid;
    bool            tid_valid;
    pthread_mutex_t lock;
    pthread_cond_t  cv;
    bool            armed;
    bool            detecting;      /* in an approach phase (not pull-off) */
    uint32_t        phase_gen;      /* bumped at each approach phase start */
    axes_signals_t  cycle;
    uint32_t        trig_mask;
    struct timespec trig_at;
    float           k_sigma;
    float           floor_thresh;
} home = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cv = PTHREAD_COND_INITIALIZER,
    .k_sigma = K_SIGMA_DEFAULT,
    .floor_thresh = FLOOR_DEFAULT,
};

static on_homing_rate_set_ptr on_homing_rate_set_chain;

static double ms_since (const struct timespec *t0)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)(t.tv_sec - t0->tv_sec) * 1e3 +
           (double)(t.tv_nsec - t0->tv_nsec) / 1e6;
}

static void st_accel_ctl (const char *path)
{
    FILE *f = fopen(path, "w");
    if (f) {
        fputs(ST_ACCEL_DEV, f);
        fclose(f);
    }
}

/* Abort the homing cycle from the monitor thread: EXEC_RESET is what the
 * homing loop treats as a first-class failure (safe stop + alarm).
 * system_set_exec_state_flag routes through the core-locked atomic
 * helper, so this is safe from any thread. */
static void abort_homing (const char *why)
{
    fprintf(stderr, "homing: %s - aborting cycle\n", why);
    system_set_exec_state_flag(EXEC_RESET);
}

static int sensor_open (void)
{
    st_accel_ctl(ST_ACCEL_UNBIND);
    int fd = open(HOME_I2C_BUS, O_RDWR);
    if (fd < 0)
        return -1;
    uint8_t cfg[2] = { REG_CTRL1, CTRL1_800HZ_XYZ };
    if (ioctl(fd, I2C_SLAVE, HOME_I2C_ADDR) < 0 ||
        write(fd, cfg, 2) != 2) {
        close(fd);
        return -1;
    }
    return fd;
}

static int sensor_read (int fd, int16_t out[3])
{
    uint8_t reg = REG_OUT_X_L;
    uint8_t buf[6];
    if (write(fd, &reg, 1) != 1 || read(fd, buf, 6) != 6)
        return -1;
    out[0] = (int16_t)(buf[0] | (buf[1] << 8));
    out[1] = (int16_t)(buf[2] | (buf[3] << 8));
    out[2] = (int16_t)(buf[4] | (buf[5] << 8));
    return 0;
}

static void *monitor (void *arg)
{
    (void)arg;

    for (;;) {
        pthread_mutex_lock(&home.lock);
        while (!home.armed)
            pthread_cond_wait(&home.cv, &home.lock);
        pthread_mutex_unlock(&home.lock);

        int fd = sensor_open();
        if (fd < 0) {
            abort_homing("head accelerometer unavailable");
            st_accel_ctl(ST_ACCEL_BIND);
            /* wait out the disarm so we don't spin */
            pthread_mutex_lock(&home.lock);
            while (home.armed)
                pthread_cond_wait(&home.cv, &home.lock);
            pthread_mutex_unlock(&home.lock);
            continue;
        }

        float ema[3];
        bool ema_seeded = false;
        double sum = 0.0, sum2 = 0.0;
        unsigned n = 0;
        float thresh = 0.0f;
        int over = 0, errors = 0;
        bool fired = false;
        uint32_t seen_gen = 0;
        struct timespec phase_at;
        clock_gettime(CLOCK_MONOTONIC, &phase_at);

        for (;;) {
            /* cycle_mask must track home.cycle live: $H runs multiple
             * cycles under one continuous arm (enable is re-called with
             * the next cycle's mask without a disarm in between) */
            pthread_mutex_lock(&home.lock);
            bool armed = home.armed;
            bool detecting = home.detecting;
            uint32_t gen = home.phase_gen;
            uint32_t cycle_mask = home.cycle.mask;
            pthread_mutex_unlock(&home.lock);
            if (!armed)
                break;

            /* Watchdog: a contact the core never acts on (axis mismatch,
             * wedged state) must not grind to the over-travel alarm. */
            if (fired && detecting && gen == seen_gen &&
                ms_since(&home.trig_at) > 5000.0) {
                abort_homing("contact signal not acted on");
                break;
            }

            if (gen != seen_gen) {
                /* new approach phase: fresh baseline and threshold */
                seen_gen = gen;
                clock_gettime(CLOCK_MONOTONIC, &phase_at);
                sum = sum2 = 0.0;
                n = 0;
                thresh = 0.0f;
                over = 0;
                fired = false;
            }

            int16_t v[3];
            if (sensor_read(fd, v)) {
                if (++errors >= MAX_I2C_ERRORS) {
                    abort_homing("head accelerometer read errors");
                    break;
                }
                usleep(SAMPLE_US);
                continue;
            }
            errors = 0;

            if (!ema_seeded) {
                ema_seeded = true;
                for (int i = 0; i < 3; i++)
                    ema[i] = (float)v[i];
            }
            float dev = 0.0f;
            for (int i = 0; i < 3; i++) {
                dev += fabsf((float)v[i] - ema[i]);
                ema[i] += EMA_ALPHA * ((float)v[i] - ema[i]);
            }

            double t = ms_since(&phase_at);
            if (!detecting || fired || t < RAMP_MS) {
                /* pull-off phase, already fired, or approach ramp:
                 * keep the gravity tracker settled, nothing else */
            } else if (t < RAMP_MS + LEARN_MS) {
                sum += dev;
                sum2 += (double)dev * dev;
                n++;
            } else {
                if (thresh == 0.0f && n > 0) {
                    float mean = (float)(sum / n);
                    float sd = sqrtf((float)(sum2 / n) - mean * mean);
                    thresh = fmaxf(mean + home.k_sigma * sd,
                                   home.floor_thresh);
                    fprintf(stderr, "homing: approach axes=%02x "
                            "thresh=%.0f (base mean %.0f sd %.0f)\n",
                            (unsigned)cycle_mask, thresh, mean, sd);
                    if (mean > CONTACT_BASELINE) {
                        /* started pressed against the rail */
                        pthread_mutex_lock(&home.lock);
                        home.trig_mask = cycle_mask;
                        clock_gettime(CLOCK_MONOTONIC, &home.trig_at);
                        pthread_mutex_unlock(&home.lock);
                        fired = true;
                        fprintf(stderr, "homing: contact at approach "
                                "start (grinding baseline)\n");
                    }
                }
                if (thresh > 0.0f && dev > thresh) {
                    if (++over >= CONFIRM_SAMPLES) {
                        pthread_mutex_lock(&home.lock);
                        home.trig_mask = cycle_mask;
                        clock_gettime(CLOCK_MONOTONIC, &home.trig_at);
                        pthread_mutex_unlock(&home.lock);
                        fired = true;
                        fprintf(stderr,
                                "homing: contact (dev %.0f)\n", dev);
                    }
                } else {
                    over = 0;
                }
            }
            usleep(SAMPLE_US);
        }

        close(fd);
        st_accel_ctl(ST_ACCEL_BIND);

        /* if we aborted, wait for the core to disarm before re-arming */
        pthread_mutex_lock(&home.lock);
        while (home.armed)
            pthread_cond_wait(&home.cv, &home.lock);
        pthread_mutex_unlock(&home.lock);
    }
    return NULL;
}

/* Phase tracking: each Seek/Locate start opens a fresh detection
 * session; Pulloff suspends detection (its reversal/stop jerks must not
 * read as contact) and clears any pending trigger so the pull-off
 * completion check sees the switch released. */
static void homingRateSet (axes_signals_t axes, coord_data_t *feedrate,
                           homing_mode_t mode)
{
    if (on_homing_rate_set_chain)
        on_homing_rate_set_chain(axes, feedrate, mode);

    pthread_mutex_lock(&home.lock);
    if (mode == HomingMode_Pulloff) {
        home.detecting = false;
        home.trig_mask = 0;
    } else {
        home.detecting = true;
        home.phase_gen++;
    }
    pthread_mutex_unlock(&home.lock);
}

void gfhome_init (void)
{
    const char *v;
    if ((v = getenv("GFHOME_KSIGMA")) != NULL && atof(v) > 0.0)
        home.k_sigma = (float)atof(v);
    if ((v = getenv("GFHOME_FLOOR")) != NULL && atof(v) > 0.0)
        home.floor_thresh = (float)atof(v);

    on_homing_rate_set_chain = grbl.on_homing_rate_set;
    grbl.on_homing_rate_set = homingRateSet;

    if (pthread_create(&home.tid, NULL, monitor, NULL) == 0)
        home.tid_valid = true;
    else
        fprintf(stderr, "homing: monitor thread failed to start\n");
}

void gfhome_enable (bool on, axes_signals_t homing_cycle)
{
    (void)on;   /* no physical limit inputs exist to hard-limit with */

    if (homing_cycle.mask != 0 && !home.tid_valid) {
        abort_homing("no accelerometer monitor thread");
        return;
    }

    pthread_mutex_lock(&home.lock);
    home.cycle = homing_cycle;
    home.armed = home.tid_valid && homing_cycle.mask != 0;
    home.detecting = false;     /* until the first approach phase starts */
    home.trig_mask = 0;
    pthread_cond_broadcast(&home.cv);
    pthread_mutex_unlock(&home.lock);
}

limit_signals_t gfhome_get_state (void)
{
    limit_signals_t signals = {0};

    pthread_mutex_lock(&home.lock);
    if (home.trig_mask && ms_since(&home.trig_at) <= TRIG_HOLD_MS)
        signals.min.mask = home.trig_mask;
    else
        home.trig_mask = 0;
    pthread_mutex_unlock(&home.lock);

    return signals;
}
