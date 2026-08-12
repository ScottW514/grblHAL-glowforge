/*
  glowforge_io.c - Glowforge kernel-driver sysfs / pulse-device access

  Part of grblHAL-glowforge.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <ctype.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

#include "glowforge_io.h"

#define GF_SYSFS "/sys/glowforge/"

/* PIC stepper currents from the factory print header (XSrc/XShc, YSrc/YShc;
 * identical across every captured job, 2018 and 2026 firmware alike). The
 * X/Y scales differ by design - these are the factory's own DAC values. */
#define X_CURRENT_RUN  "135"
#define X_CURRENT_HOLD "33"
#define Y_CURRENT_RUN  "22"
#define Y_CURRENT_HOLD "5"

/* Null-sink instances must never touch the machine: on a dev host the
 * sysfs opens simply fail, but on the board a "no hardware I/O" test
 * process would otherwise drive the real laser latch and analog config.
 * Writes are gated until the instance declares hardware ownership. */
static bool gfio_hw = false;

void gfio_set_hw (bool active)
{
    gfio_hw = active;
}

int gfio_wr_attr (const char *attr, const char *val)
{
    char path[128];
    int fd, ret;
    if(!gfio_hw)
        return 0;
    snprintf(path, sizeof(path), GF_SYSFS "%s", attr);
    if((fd = open(path, O_WRONLY)) < 0)
        return -1;
    ret = write(fd, val, strlen(val)) < 0 ? -1 : 0;
    close(fd);
    return ret;
}

int gfio_rd_attr (const char *attr, char *buf, size_t len)
{
    char path[128];
    int fd;
    ssize_t n;
    snprintf(path, sizeof(path), GF_SYSFS "%s", attr);
    if((fd = open(path, O_RDONLY)) < 0)
        return -1;
    n = read(fd, buf, len - 1);
    close(fd);
    if(n < 0)
        return -1;
    while(n > 0 && buf[n - 1] == '\n')
        n--;
    buf[n] = '\0';
    return 0;
}

static int open_pulse_dev (const char *path, int lock_flags)
{
    int fd;

    if((fd = open(path, O_WRONLY)) < 0)
        return -1;

    if(flock(fd, lock_flags) != 0) {
        close(fd);
        return -1;
    }

    return fd;
}

/* Broker mode: the forgectrl supervisor holds /dev/glowforge open for
 * its lifetime (the device is exclusive-open) and hands the writer a
 * dup at spawn, named by GF_PULSE_FD. The flock rides the shared open
 * file description - the relock below is a no-op on it - and closing
 * this fd is never the final close while the supervisor lives, so
 * handovers stop cycling the 40 V rail. The supervisor is the
 * dead-man for its writers; the kernel dead-man still backstops the
 * whole description. */
static int inherited_pulse_fd (void)
{
    const char *v = getenv("GF_PULSE_FD");
    if(v == NULL || *v == '\0')
        return -1;
    int fd = atoi(v);
    if(fd < 0 || fcntl(fd, F_GETFL) < 0)
        return -1;
    return fd;
}

bool gfio_pulse_inherited (void)
{
    return inherited_pulse_fd() >= 0;
}

int gfio_open_pulse_dev (const char *path)
{
    int fd = inherited_pulse_fd();
    if(fd >= 0) {
        flock(fd, LOCK_EX);
        return fd;
    }
    return open_pulse_dev(path, LOCK_EX);
}

int gfio_open_pulse_dev_nb (const char *path)
{
    int fd = inherited_pulse_fd();
    if(fd >= 0) {
        flock(fd, LOCK_EX);
        return fd;
    }
    return open_pulse_dev(path, LOCK_EX | LOCK_NB);
}

void gfio_analog_config (void)
{
    gfio_wr_attr("cnc/laser_latch", "1");
    gfio_wr_attr("cnc/x_mode", "8");
    gfio_wr_attr("cnc/y_mode", "8");
    gfio_wr_attr("cnc/x_decay", "1");
    gfio_wr_attr("cnc/y_decay", "1");
    gfio_wr_attr("cnc/motor_lock", "8");
    gfio_currents_hold();
}

void gfio_currents_run (void)
{
    gfio_wr_attr("pic/x_step_current", X_CURRENT_RUN);
    gfio_wr_attr("pic/y_step_current", Y_CURRENT_RUN);
}

void gfio_currents_hold (void)
{
    gfio_wr_attr("pic/x_step_current", X_CURRENT_HOLD);
    gfio_wr_attr("pic/y_step_current", Y_CURRENT_HOLD);
}

/* --- the shared machine config ("key = value", '#' comments) --------- */

#define CONF_DEFAULT "/data/forgefirm.conf"

static const char *conf_path (void)
{
    const char *p = getenv("GFHOME_CONF");
    return (p && *p) ? p : CONF_DEFAULT;
}

int gfio_conf_read (const char *key, char *val, size_t len)
{
    FILE *f = fopen(conf_path(), "r");
    if(f == NULL)
        return -1;

    char line[256];
    int found = -1;
    while(fgets(line, sizeof(line), f)) {
        char *p = line;
        while(isspace((unsigned char)*p))
            p++;
        if(*p == '#' || *p == '\0')
            continue;
        char *eq = strchr(p, '=');
        if(eq == NULL)
            continue;
        char *k_end = eq;
        while(k_end > p && isspace((unsigned char)k_end[-1]))
            k_end--;
        *k_end = '\0';
        if(strcmp(p, key))
            continue;
        char *v = eq + 1;
        while(isspace((unsigned char)*v))
            v++;
        char *v_end = v + strlen(v);
        while(v_end > v && isspace((unsigned char)v_end[-1]))
            v_end--;
        *v_end = '\0';
        snprintf(val, len, "%s", v);
        found = 0;                  /* keep scanning: last occurrence wins */
    }
    fclose(f);
    return found;
}

float gfio_conf_read_float (const char *key, float fallback)
{
    char val[32], *end;
    if(gfio_conf_read(key, val, sizeof(val)) != 0)
        return fallback;
    float f = strtof(val, &end);
    return end == val ? fallback : f;
}
