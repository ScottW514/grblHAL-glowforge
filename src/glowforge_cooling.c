/*
  glowforge_cooling.c - fan / coolant management for the Glowforge board

  Part of grblHAL-glowforge. Mirrors the factory firmware's job-state fan
  scheme (gfhardware Machine._config_from_pulse): run values while
  cutting, a cooldown period after, idle values otherwise.

  Every value is the factory's own, taken from captured pulse-file
  headers (_RESOURCES/*.info; decoded via the gfutilities settings map):
  air assist AAid=204 / AArd=1023 / AAwd=1023 (0-1023), exhaust EFrd=65535,
  intake IFrd=43278 (0-65535), purge air on (PAon=1), coolant pump on.
  Motion-only factory jobs (hunt/motion) ran fans at idle - matched here:
  the cut profile engages only on M8 (coolant flood), which senders like
  LightBurn emit per-layer as "air assist". The eventual laser milestone
  will also key on spindle state; the M8 path gives the fans full
  exercise before any beam exists.

  Water temperature (upstream sensor, pic/water_temp_2, C = raw *
  -0.09653 + 94) is checked at 1 Hz; exceeding the factory run ceiling
  (~31 C, CMrx) raises a controller warning once per excursion. The
  laser milestone will turn that into a hard gate for firing.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "glowforge_cooling.h"
#include "glowforge_io.h"

#include "grbl/hal.h"
#include "grbl/report.h"

/* Factory fan values (pulse-header ground truth). */
#define AIR_ASSIST_IDLE "204"
#define AIR_ASSIST_RUN  "1023"
#define EXHAUST_IDLE    "0"
#define EXHAUST_RUN     "65535"
#define INTAKE_IDLE     "0"
#define INTAKE_RUN      "43278"

/* Cooldown: factory used a cloud-configured delay; default 15 s here
 * (env GFCOOL_COOLDOWN_S). Cooldown fan values equal run values
 * (AAwd/EFwd/IFwd matched AArd/EFrd/IFrd in every captured header). */
#define COOLDOWN_S_DEFAULT 15

/* Water temp warning threshold, degrees C (factory run ceiling CMrx). */
#define WATER_WARN_C 31.0f

typedef enum {
    Cool_Idle = 0,
    Cool_Run,
    Cool_Cooldown,
} cool_state_t;

static cool_state_t cool_state = Cool_Idle;
static coolant_state_t coolant_reported = {0};
static uint32_t cooldown_s = COOLDOWN_S_DEFAULT;
static double cooldown_until;
static double next_temp_check;
static bool temp_warned = false;

static double wall_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static void fans_idle (void)
{
    gfio_wr_attr("head/air_assist_pwm", AIR_ASSIST_IDLE);
    gfio_wr_attr("thermal/exhaust_pwm", EXHAUST_IDLE);
    gfio_wr_attr("thermal/intake_pwm", INTAKE_IDLE);
}

static void fans_run (void)
{
    gfio_wr_attr("head/air_assist_pwm", AIR_ASSIST_RUN);
    gfio_wr_attr("thermal/exhaust_pwm", EXHAUST_RUN);
    gfio_wr_attr("thermal/intake_pwm", INTAKE_RUN);
}

void gfcool_init (void)
{
    const char *opt;

    if((opt = getenv("GFCOOL_COOLDOWN_S")))
        cooldown_s = (uint32_t)atoi(opt);

    gfio_wr_attr("thermal/water_pump_on", "1");
    gfio_wr_attr("thermal/tec_on", "0");
    gfio_wr_attr("head/purge_air", "1");
    fans_idle();

    next_temp_check = wall_s() + 5.0;
}

void gfcool_coolant_set (coolant_state_t state)
{
    coolant_reported = state;

    if(state.flood) {
        cool_state = Cool_Run;
        fans_run();
    } else if(cool_state == Cool_Run) {
        /* Cut just ended: keep the air moving to clear smoke, then idle.
         * Cooldown fan values equal run values per the factory headers. */
        cool_state = Cool_Cooldown;
        cooldown_until = wall_s() + (double)cooldown_s;
    }
}

coolant_state_t gfcool_coolant_get (void)
{
    return coolant_reported;
}

void gfcool_poll (void)
{
    double now = wall_s();

    if(cool_state == Cool_Cooldown && now >= cooldown_until) {
        cool_state = Cool_Idle;
        fans_idle();
    }

    if(now >= next_temp_check) {
        next_temp_check = now + 1.0;

        char buf[16];
        if(gfio_rd_attr("pic/water_temp_2", buf, sizeof(buf)) == 0) {
            float c = (float)atoi(buf) * -0.09653f + 94.0f;
            if(c > WATER_WARN_C && !temp_warned) {
                temp_warned = true;
                char msg[64];
                snprintf(msg, sizeof(msg), "coolant temperature high: %.1f C", c);
                report_message(msg, Message_Warning);
                fprintf(stderr, "gfcool: %s\n", msg);
            } else if(c < WATER_WARN_C - 1.0f)
                temp_warned = false;    /* 1 C hysteresis re-arms the warning */
        }
    }
}
