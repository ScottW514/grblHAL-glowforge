/*
 * fflog.c - ForgeFIRM syslog emitter (see fflog.h)
 * Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
 * SPDX-License-Identifier: MIT
 *
 * Vendored verbatim into the other ForgeFIRM C daemons; keep the
 * copies identical.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "fflog.h"

#include <ctype.h>
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define FFLOG_SOCK      "/dev/log"
#define FFLOG_CONF_DEF  "/data/forgefirm.conf"
#define FFLOG_FACILITY  LOG_DAEMON
#define FFLOG_MSG_MAX   2048
#define RECONNECT_MIN_S 1.0

static char ident[32] = "fflog";
static int level = LOG_INFO;
static int echo_stderr = 0;
static int sock = -1;
static double last_connect_try = -1e9;
static unsigned long dropped = 0;
static unsigned long dropped_reported = 0;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;

static double mono_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}

int fflog_parse_level(const char *name, int *out)
{
    static const struct { const char *n; int l; } tab[] = {
        { "off", -1 },
        { "error", LOG_ERR }, { "err", LOG_ERR },
        { "warning", LOG_WARNING }, { "warn", LOG_WARNING },
        { "notice", LOG_NOTICE },
        { "info", LOG_INFO },
        { "debug", LOG_DEBUG },
    };
    if (!name)
        return -1;
    for (size_t i = 0; i < sizeof(tab) / sizeof(*tab); i++)
        if (!strcasecmp(name, tab[i].n)) {
            *out = tab[i].l;
            return 0;
        }
    return -1;
}

/* Trim leading/trailing whitespace in place. */
static char *trim(char *s)
{
    while (isspace((unsigned char)*s))
        s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1]))
        e--;
    *e = '\0';
    return s;
}

/* The emit level is the more verbose of the disk and remote levels:
 * rsyslog filters each destination down; the process only has to
 * produce enough for the noisier one. Unknown values fall back to the
 * default so a hand-edited file cannot silence a logger by mistake. */
static int level_from_conf(const char *path, const char *id)
{
    char kd[64], kr[64];
    snprintf(kd, sizeof(kd), "log_%s_disk", id);
    snprintf(kr, sizeof(kr), "log_%s_remote", id);
    int ld = LOG_INFO, lr = -1, have_d = 0, have_r = 0;
    FILE *f = fopen(path, "r");
    if (f) {
        char line[512];
        while (fgets(line, sizeof(line), f)) {
            char *eq = strchr(line, '=');
            if (!eq || line[0] == '#')
                continue;
            *eq = '\0';
            char *k = trim(line), *v = trim(eq + 1);
            int l;
            if (!strcmp(k, kd) && fflog_parse_level(v, &l) == 0) {
                ld = l;
                have_d = 1;
            } else if (!strcmp(k, kr) && fflog_parse_level(v, &l) == 0) {
                lr = l;
                have_r = 1;
            }
        }
        fclose(f);
    }
    (void)have_d;
    (void)have_r;
    return ld > lr ? ld : lr;
}

/* Called with mu held. */
static void connect_locked(void)
{
    double now = mono_s();
    if (now - last_connect_try < RECONNECT_MIN_S)
        return;
    last_connect_try = now;
    if (sock >= 0) {
        close(sock);
        sock = -1;
    }
    int s = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (s < 0)
        return;
    struct sockaddr_un sa;
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    const char *path = getenv("FFLOG_SOCK");     /* host-test hook */
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s",
             path && *path ? path : FFLOG_SOCK);
    if (connect(s, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        close(s);
        return;
    }
    sock = s;
}

void fflog_init(const char *id)
{
    snprintf(ident, sizeof(ident), "%s", id ? id : "fflog");
    const char *conf = getenv("FFLOG_CONF");
    level = level_from_conf(conf && *conf ? conf : FFLOG_CONF_DEF, ident);
    const char *env = getenv("FFLOG_LEVEL");
    int l;
    if (env && fflog_parse_level(env, &l) == 0)
        level = l;
    const char *e = getenv("FFLOG_STDERR");
    echo_stderr = (e && *e && strcmp(e, "0")) || isatty(2);
    pthread_mutex_lock(&mu);
    connect_locked();
    pthread_mutex_unlock(&mu);
}

int fflog_level(void)
{
    return level;
}

void fflog_set_level(int l)
{
    level = l;
}

unsigned long fflog_dropped(void)
{
    return __atomic_load_n(&dropped, __ATOMIC_RELAXED);
}

/* One datagram, never blocking. Returns 0 if queued, -1 if dropped. */
static int send_raw(const char *pkt, size_t len)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        int s = sock;
        if (s < 0) {
            pthread_mutex_lock(&mu);
            connect_locked();
            s = sock;
            pthread_mutex_unlock(&mu);
            if (s < 0)
                return -1;
        }
        if (send(s, pkt, len, MSG_NOSIGNAL | MSG_DONTWAIT) == (ssize_t)len)
            return 0;
        int err = errno;
        if (err == EAGAIN || err == EWOULDBLOCK || err == ENOBUFS ||
            err == EMSGSIZE || err == EINTR)
            return -1;            /* full: drop, never wait */
        /* Peer gone (log daemon restarted): reconnect once and retry. */
        pthread_mutex_lock(&mu);
        if (sock == s) {
            last_connect_try = -1e9;
            connect_locked();
        }
        pthread_mutex_unlock(&mu);
    }
    return -1;
}

static size_t header(char *buf, size_t cap, int prio)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    char ts[32];
    /* RFC 3164 timestamp: "Mmm dd hh:mm:ss" (day space-padded). */
    strftime(ts, sizeof(ts), "%b %e %H:%M:%S", &tm);
    int n = snprintf(buf, cap, "<%d>%s %s[%d]: ",
                     FFLOG_FACILITY | prio, ts, ident, (int)getpid());
    return n < 0 ? 0 : (size_t)n < cap ? (size_t)n : cap - 1;
}

void fflog(int prio, const char *fmt, ...)
{
    if (prio < 0)
        prio = 0;
    if (prio > LOG_DEBUG)
        prio = LOG_DEBUG;
    if (prio > level)
        return;

    char pkt[FFLOG_MSG_MAX];
    size_t off = header(pkt, sizeof(pkt), prio);
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(pkt + off, sizeof(pkt) - off, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    size_t len = off + (size_t)n;
    if (len >= sizeof(pkt))
        len = sizeof(pkt) - 1;
    while (len > off && (pkt[len - 1] == '\n' || pkt[len - 1] == '\r'))
        len--;
    pkt[len] = '\0';

    if (echo_stderr)
        fprintf(stderr, "%s: %s\n", ident, pkt + off);

    if (send_raw(pkt, len) != 0) {
        __atomic_fetch_add(&dropped, 1, __ATOMIC_RELAXED);
        return;
    }
    /* Account for a gap once the path is healthy again. */
    unsigned long d = __atomic_load_n(&dropped, __ATOMIC_RELAXED);
    if (d != dropped_reported) {
        dropped_reported = d;
        char note[FFLOG_MSG_MAX];
        size_t o = header(note, sizeof(note), LOG_WARNING);
        int m = snprintf(note + o, sizeof(note) - o,
                         "fflog: %lu message(s) dropped (syslog socket "
                         "unavailable or full)", d);
        if (m > 0)
            (void)send_raw(note, o + (size_t)m);
    }
}
