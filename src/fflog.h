/*
 * fflog.h - ForgeFIRM syslog emitter (shared across the ForgeFIRM daemons)
 * Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
 * SPDX-License-Identifier: MIT
 *
 * Every ForgeFIRM process logs through the system syslog socket
 * (/dev/log); rsyslog is the only file writer and routes each program
 * to its own directory under /data/log/forgefirm. This emitter differs
 * from glibc syslog(3) in one deliberate way: the socket is
 * NON-BLOCKING and a message that cannot be queued is dropped and
 * counted. A unix datagram socket exerts backpressure on its sender, so
 * a stalled log daemon could otherwise park a controller thread - on
 * a machine with a laser, a lost log line is the correct failure mode.
 *
 * Levels are the syslog severities (<syslog.h> LOG_ERR .. LOG_DEBUG).
 * The emit level comes from the shared machine config
 * (/data/forgefirm.conf): log_<ident>_disk and log_<ident>_remote,
 * each off|error|warning|notice|info|debug - the process emits at the
 * more verbose of the two and rsyslog filters per destination. Read
 * once at start; a change takes effect at the next process start
 * (in practice: reboot).
 *
 * Messages are echoed to stderr as well when stderr is a terminal or
 * FFLOG_STDERR=1 is set (bench runs, host-side test harnesses); under
 * the supervisor stderr is a pipe into a syslog relay, so no echo.
 * FFLOG_LEVEL overrides the configured level, FFLOG_CONF the config
 * path, FFLOG_SOCK the socket path (host tests against a private
 * rsyslog instance).
 *
 * This file is vendored verbatim into the other ForgeFIRM C daemons
 * (grblHAL-glowforge); keep the copies identical.
 */
#ifndef FFLOG_H
#define FFLOG_H

#include <syslog.h>       /* LOG_ERR, LOG_WARNING, LOG_NOTICE, LOG_INFO, LOG_DEBUG */

/* Initialize: ident is the program name rsyslog routes on (also the
 * key fragment: log_<ident>_disk / _remote). Reads the level from the
 * config file (FFLOG_CONF env overrides the path; default
 * /data/forgefirm.conf). Safe to call once, before any threads. */
void fflog_init(const char *ident);

/* Current emit level (a LOG_* severity; messages above it are not
 * formatted at all) and an override, e.g. for a --verbose flag. */
int  fflog_level(void);
void fflog_set_level(int level);

/* Parse a level name (off|error|warning|notice|info|debug) into a
 * LOG_* severity; "off" yields -1 (nothing emits). Returns 0 on
 * success, -1 on an unknown name. */
int  fflog_parse_level(const char *name, int *level);

/* Emit. prio is a LOG_* severity. Trailing newlines are stripped. */
void fflog(int prio, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* Messages dropped because the socket was full or unavailable. */
unsigned long fflog_dropped(void);

#endif /* FFLOG_H */
