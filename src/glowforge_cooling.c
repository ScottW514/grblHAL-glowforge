/*
  glowforge_cooling.c - cooling-service client for the Glowforge board

  Part of grblHAL-glowforge. The cooling engine - fan/pump/TEC/heater
  profiles, coolant-flow verification, over-temp policy - lives in
  forgectrl (the machine-services daemon); the shared contract is
  forgectrl docs/SERVICES.md. This client:

  - reports job state to the engine: POST /cool/state (127.0.0.1,
    FORGECTRL_PORT or 8080) with mode=idle|run|cooldown and the armed
    flag - level-triggered, re-sent every ~1 s from gfcool_poll and
    immediately on every change, so a lost report self-heals. The
    effective run window is the sender's M8/M9 OR'd with the laser
    armed window: fire must never run without the cut airflow and the
    flow interrogation that only lives inside a run session.

  - reads the engine's verdict from /run/forgefirm/cooling.state and
    enforces it in-process: gfcool_fire_ok() gates the laser (the
    stepper producer thread reads a cached flag with a monotonic
    freshness deadline - no file IO on that path), a hold verdict
    takes a real feed hold (jogs are canceled instead - grblHAL never
    holds a jog), and resume_ok auto-resumes a hold this client took.
    A missing or stale verdict (ts_mono older than 2 s) is treated as
    fire_ok=false, hold=true: the engine being gone must look exactly
    like a fault.

  - EMERGENCY FALLBACK: if the verdict goes stale while the laser is
    armed, the engine is provably absent and nobody else will move the
    fans - so this client writes the factory run duties directly to
    sysfs once, holds, and stands down. This is the one sanctioned
    exception to the engine's single-writer ownership; the duties are
    compiled in (no config dependency - the same failure may take the
    config file).

  The hardware AND-gate (OK_2_FIRE) remains the safety boundary;
  everything here is equipment protection and defense in depth.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "glowforge_cooling.h"
#include "glowforge_io.h"

#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"

/* After the grbl headers: glibc's stat.h (via fcntl.h) defines an
 * st_mtime macro that would otherwise mangle the field of that name in
 * vfs.h. */
#include <fcntl.h>

#define VERDICT_FILE   "/run/forgefirm/cooling.state"
/* The verdict path is overridable for the host-side stream harness
 * (GF_VERDICT_FILE), which publishes its own verdicts; a wrong value
 * only ever blocks fire (missing file = stale = blocked). */
static const char *verdict_file = VERDICT_FILE;
/* Reader staleness window per the contract: a verdict older than this
 * is no verdict. The engine publishes at 1 Hz. */
#define VERDICT_MAX_AGE_MS 2000
/* Report cadence (level-triggered refresh). */
#define REPORT_PERIOD_MS   1000
/* Socket timeouts for the report POST: localhost, so anything slower
 * than this means forgectrl is wedged - drop the report and move on
 * (the level-triggered refresh retries in a second). */
#define REPORT_TIMEOUT_MS  250

/* Factory run fan duties for the emergency fallback (pulse-header
 * ground truth, mirrored in the contract doc). */
#define FB_AIR_ASSIST "1023"
#define FB_EXHAUST    "65535"
#define FB_INTAKE     "43278"

static coolant_state_t coolant_reported = {0};
static bool laser_on_window = false;    /* armed (glowforge_laser.c) */
static bool was_run = false;            /* for the cooldown report */

/* Verdict cache. fire_ok is read from the stepper producer thread; the
 * deadline comparison makes a stalled poll thread read as fire-blocked.
 * Ordering: the flags are published BEFORE the deadline (release), and
 * readers take the deadline first (acquire) - so a reader can never
 * pair a refreshed deadline with the previous, possibly laxer flags. */
static _Atomic bool v_fire_ok = false;
static _Atomic bool v_hold = false;
static _Atomic bool v_resume_ok = false;
static _Atomic uint32_t v_fresh_until_ms = 0;
static char v_reason[112];
static char v_reason_shown[112];

static bool hold_ours = false;
static bool jog_warned = false;
static bool fallback_done = false;
static bool stale_warned = false;

static uint32_t next_report_ms = 0;
static uint32_t next_verdict_ms = 0;
static int http_port = 8080;

static uint32_t mono_ms (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}

static void warn (const char *msg)
{
    report_message(msg, Message_Warning);
    fprintf(stderr, "gfcool: %s\n", msg);
}

/* ------------------------------------------------------- job reports */

/* One fire-and-forget POST, on the reporter thread only. localhost:
 * connection refused returns immediately when forgectrl is down. The
 * connect is non-blocking and bounded by poll - SO_SNDTIMEO does not
 * cover connect(), and a wedged listener (full accept backlog) blocks
 * a plain connect through minutes of SYN retries. */
static void report_send (const char *mode, bool armed)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if(fd < 0)
        return;

    struct timeval tv = { 0, REPORT_TIMEOUT_MS * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)http_port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    bool up = connect(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0;
    if(!up && errno == EINPROGRESS) {
        struct pollfd pfd = { .fd = fd, .events = POLLOUT };
        int err = 0;
        socklen_t el = sizeof(err);
        up = poll(&pfd, 1, REPORT_TIMEOUT_MS) == 1 &&
              getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el) == 0 && err == 0;
    }
    if(up) {
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
        char req[192];
        int n = snprintf(req, sizeof(req),
            "POST /cool/state?mode=%s&armed=%d HTTP/1.1\r\n"
            "Host: 127.0.0.1\r\nConnection: close\r\n"
            "Content-Length: 0\r\n\r\n", mode, armed ? 1 : 0);
        if(write(fd, req, (size_t)n) == n) {
            char resp[128];
            (void)!read(fd, resp, sizeof(resp));    /* let the ack land */
        }
    }
    close(fd);
}

/* Reporter thread: report_send can still block for the socket timeouts,
 * which is far past the ~10 ms the protocol thread can afford inside a
 * cycle (a stalled protocol thread starves the segment lookahead while
 * the shipper keeps cutting). Level-triggered hand-off: the protocol
 * thread deposits the latest mode+armed and kicks; intermediate states
 * may be skipped, only the newest matters. */
static pthread_mutex_t rep_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rep_cv = PTHREAD_COND_INITIALIZER;
static char rep_mode[12] = "idle";
static bool rep_armed = false;
static unsigned rep_seq = 0;

static void *reporter_thread (void *arg)
{
    (void)arg;

    unsigned seen = 0;
    char mode[12];
    bool armed;

    pthread_mutex_lock(&rep_mx);
    for(;;) {
        while(rep_seq == seen)
            pthread_cond_wait(&rep_cv, &rep_mx);
        seen = rep_seq;
        strcpy(mode, rep_mode);
        armed = rep_armed;
        pthread_mutex_unlock(&rep_mx);

        report_send(mode, armed);

        pthread_mutex_lock(&rep_mx);
    }

    return NULL;
}

static const char *report_mode (void)
{
    if(coolant_reported.flood || laser_on_window)
        return "run";
    /* After a run: the engine owns the cooldown phases; "cooldown"
     * tells it the job just ended vs. a machine that was never
     * running. Cleared once grbl is back at idle. */
    if(was_run && (state_get() & (STATE_CYCLE | STATE_HOLD)))
        return "cooldown";
    return "idle";
}

static void report_now (void)
{
    const char *mode = report_mode();
    if(!strcmp(mode, "idle"))
        was_run = false;

    pthread_mutex_lock(&rep_mx);
    strncpy(rep_mode, mode, sizeof(rep_mode) - 1);
    rep_mode[sizeof(rep_mode) - 1] = '\0';
    rep_armed = laser_on_window;
    rep_seq++;
    pthread_cond_signal(&rep_cv);
    pthread_mutex_unlock(&rep_mx);

    next_report_ms = mono_ms() + REPORT_PERIOD_MS;
}

/* ---------------------------------------------------------- verdict */

/* Minimal field extraction from the engine's fixed JSON (we own the
 * writer; keys are matched, order is not assumed). A key that is absent
 * takes the caller's default, which is always the fail-safe direction:
 * fire blocked, hold on, resume refused. */
static bool json_bool (const char *body, const char *key, bool dflt)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(body, pat);
    if(!p)
        return dflt;
    p += strlen(pat);
    if(!strncmp(p, "true", 4))
        return true;
    if(!strncmp(p, "false", 5))
        return false;
    return dflt;
}

static bool json_num (const char *body, const char *key, double *out)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(body, pat);
    if(!p)
        return false;
    *out = atof(p + strlen(pat));
    return true;
}

static void json_str (const char *body, const char *key, char *out, size_t len)
{
    char pat[24];
    out[0] = '\0';
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(body, pat);
    if(!p)
        return;
    p += strlen(pat);
    const char *e = strchr(p, '"');
    size_t n = e ? (size_t)(e - p) : 0;
    if(n >= len)
        n = len - 1;
    memcpy(out, p, n);
    out[n] = '\0';
}

/* Read + parse the verdict file; refresh the cached flags. A missing
 * or unparsable file, or a stale ts_mono, leaves the cache to expire:
 * fire blocked, hold on. */
static void verdict_read (void)
{
    char body[1024];
    FILE *f = fopen(verdict_file, "r");
    if(!f)
        return;
    size_t n = fread(body, 1, sizeof(body) - 1, f);
    fclose(f);
    body[n] = '\0';

    /* Only a complete document is trusted: the publisher writes the
     * whole object and renames it into place, so a body without its
     * closing brace is a torn or oversized read, not a verdict. */
    if(n == 0 || memchr(body, '}', n) == NULL)
        return;

    double ts_mono;
    if(!json_num(body, "ts_mono", &ts_mono))
        return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double age = (double)ts.tv_sec + ts.tv_nsec / 1e9 - ts_mono;
    if(age < 0.0 || age > VERDICT_MAX_AGE_MS / 1000.0)
        return;     /* stale publisher; let the cache expire */

    /* Flags first, deadline last (release): see the cache comment. */
    atomic_store_explicit(&v_hold, json_bool(body, "hold", true), memory_order_relaxed);
    atomic_store_explicit(&v_resume_ok, json_bool(body, "resume_ok", false), memory_order_relaxed);
    atomic_store_explicit(&v_fire_ok, json_bool(body, "fire_ok", false), memory_order_relaxed);
    json_str(body, "reason", v_reason, sizeof(v_reason));
    atomic_store_explicit(&v_fresh_until_ms,
        mono_ms() + (uint32_t)(VERDICT_MAX_AGE_MS - (uint32_t)(age * 1000.0)),
        memory_order_release);
}

static bool verdict_fresh (void)
{
    return (int32_t)(mono_ms() -
        atomic_load_explicit(&v_fresh_until_ms, memory_order_acquire)) < 0;
}

/* Stale verdict while the laser is armed: the engine is gone with the
 * laser hot. Gate + hold happen through the normal paths (freshness);
 * the fans need the one direct write nobody else can make now. */
static void fallback_fans (void)
{
    gfio_wr_attr("head/air_assist_pwm", FB_AIR_ASSIST);
    gfio_wr_attr("thermal/exhaust_pwm", FB_EXHAUST);
    gfio_wr_attr("thermal/intake_pwm", FB_INTAKE);
}

/* ---------------------------------------------------------------- api */

void gfcool_init (void)
{
    const char *v = getenv("FORGECTRL_PORT");
    if(v && atoi(v) > 0 && atoi(v) < 65536)
        http_port = atoi(v);
    const char *vf = getenv("GF_VERDICT_FILE");
    if(vf && *vf)
        verdict_file = vf;
    next_report_ms = mono_ms();
    next_verdict_ms = mono_ms();

    /* The reporter runs detached for the process lifetime. If it cannot
     * start, reports never send and the engine treats the silence as a
     * stand-down - the failure direction is safe. */
    pthread_t tid;
    pthread_attr_t a;
    pthread_attr_init(&a);
    pthread_attr_setdetachstate(&a, PTHREAD_CREATE_DETACHED);
    if(pthread_create(&tid, &a, reporter_thread, NULL) != 0)
        fprintf(stderr, "gfcool: cannot start the reporter thread\n");
    pthread_attr_destroy(&a);
}

void gfcool_coolant_set (coolant_state_t state)
{
    bool was = coolant_reported.flood || laser_on_window;
    coolant_reported = state;
    if((state.flood || laser_on_window) != was) {
        if(!was)
            was_run = true;
        report_now();
    }
}

coolant_state_t gfcool_coolant_get (void)
{
    return coolant_reported;
}

void gfcool_laser_armed (bool armed)
{
    if(laser_on_window == armed)
        return;
    bool was = coolant_reported.flood || laser_on_window;
    laser_on_window = armed;
    if((coolant_reported.flood || armed) != was) {
        if(!was)
            was_run = true;
        report_now();
    }
}

bool gfcool_fire_ok (void)
{
    /* Freshness (acquire) first: the flag read below is then at least
     * as new as the verdict that set the deadline. */
    return verdict_fresh() &&
            atomic_load_explicit(&v_fire_ok, memory_order_relaxed);
}

void gfcool_poll (void)
{
    uint32_t now = mono_ms();

    if((int32_t)(now - next_verdict_ms) >= 0) {
        next_verdict_ms = now + 1000;
        verdict_read();

        bool fresh = verdict_fresh();
        sys_state_t st = state_get();

        if(!fresh) {
            /* No verdict: fire is blocked (freshness), motion holds,
             * and an armed window gets the fallback airflow. */
            if(laser_on_window || coolant_reported.flood) {
                if(!stale_warned) {
                    stale_warned = true;
                    warn("cooling service lost - fire blocked, holding");
                    /* The engine only runs the check heater inside this
                     * window; if it died mid-check the heater is still
                     * on, and like the fallback fans this is a write
                     * nobody else will make now. */
                    gfio_wr_attr("thermal/heater_pwm", "0");
                }
                if(st == STATE_CYCLE && !hold_ours) {
                    hold_ours = true;
                    protocol_enqueue_realtime_command(CMD_FEED_HOLD);
                }
                if(laser_on_window && !fallback_done) {
                    fallback_done = true;
                    fallback_fans();
                }
            }
        } else {
            if(stale_warned) {
                stale_warned = false;
                fallback_done = false;
                report_message("cooling service restored", Message_Info);
                fprintf(stderr, "gfcool: cooling service restored\n");
            }

            /* Relay the engine's reason to the sender once per change. */
            if(v_reason[0] && strcmp(v_reason, v_reason_shown))
                warn(v_reason);
            strcpy(v_reason_shown, v_reason);

            if(v_hold) {
                if(st == STATE_CYCLE && !hold_ours) {
                    hold_ours = true;
                    protocol_enqueue_realtime_command(CMD_FEED_HOLD);
                } else if(st == STATE_JOG) {
                    /* grblHAL never holds a jog; cancel it instead. */
                    protocol_enqueue_realtime_command(CMD_JOG_CANCEL);
                    if(!jog_warned) {
                        jog_warned = true;
                        warn("jog canceled on the cooling verdict");
                    }
                }
            } else {
                jog_warned = false;
                if(hold_ours) {
                    if(!(st & STATE_HOLD))
                        hold_ours = false;  /* operator resumed or reset */
                    else if(v_resume_ok) {
                        hold_ours = false;
                        report_message("cooling verdict clean - resuming",
                                       Message_Info);
                        fprintf(stderr, "gfcool: resuming\n");
                        protocol_enqueue_realtime_command(CMD_CYCLE_START);
                    }
                }
            }
        }
    }

    /* One reporter per machine (SERVICES.md): a gfcloud homing session
     * hands the machine to gfhome, which reports its own job state -
     * this client's idle reports would fight it, resetting the engine's
     * per-job profile and thrashing the flood state per motion. */
    if((int32_t)(now - next_report_ms) >= 0 && state_get() != STATE_HOMING)
        report_now();
}
