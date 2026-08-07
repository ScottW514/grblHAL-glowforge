/*
  glowforge_io.c - Glowforge kernel-driver sysfs / pulse-device access

  Part of grblHAL-glowforge.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <fcntl.h>
#include <stdio.h>
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

int gfio_wr_attr (const char *attr, const char *val)
{
    char path[128];
    int fd, ret;
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

int gfio_open_pulse_dev (const char *path)
{
    return open_pulse_dev(path, LOCK_EX);
}

int gfio_open_pulse_dev_nb (const char *path)
{
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
