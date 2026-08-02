/*
  glowforge_cooling.c - fan / coolant management for the Glowforge board

  Part of grblHAL-glowforge. Job-state fan and coolant policy built from
  the factory's own numbers (captured pulse-file headers + settings map):
  fan duties AAid=204/AArd=1023, EFrd=65535, IFrd=43278; coolant windows
  CM* in millidegrees - run ceiling 33 C (CMrx), start/resume gate 31 C
  (CMwx), idle max 50 C. All thresholds env-adjustable.

  Policy (v2):
  - IDLE: pump on, TEC off, purge on, fans at factory idle, HEATER OFF
    (an always-on flow heater measurably warms the loop - ~1.5 C in
    minutes on this machine - eating headroom below the 31 C start gate
    for no benefit while nothing can fire).
  - RUN (M8, LightBurn's per-layer air assist): cut-profile fans AND the
    flow-verification heater at the factory 10% duty. The heater sits
    between the two water-temp sensors; flowing coolant carries its heat
    away. Measured here 2026-08-02: flow dT settles +2.5..4.0 C (never
    sustains >4.1), pump-stopped dT hits +4.6..5.7 within 20 s. Flow
    faulting arms 30 s after heater-on (dT needs ~15-20 s to establish)
    and warns at dT>4.5 C sustained 20 s (re-arms below 4.0).
  - COOLDOWN (M9): 15 s smoke-clear at run duty, then a thermal phase at
    reduced duty (fan airflow measurably cools the loop) until the
    upstream temp is back under the resume gate or a timeout expires;
    heater off for the whole cooldown (it would fight the cooling).
  - OVER-TEMP HOLD: if the upstream temp exceeds the run ceiling while a
    cycle/jog is executing, the driver issues a real feed hold (the
    factory pauses jobs the same way), forces cooling airflow, and
    auto-resumes once the temp is back under the resume gate. Senders
    see the Hold state and the [MSG:Warning:...] lines.

  The laser milestone upgrades the flow fault and the temperature gates
  from warnings/holds into hard fire interlocks.

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
#include "grbl/grbl.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"

/* Factory fan values (pulse-header ground truth). */
#define AIR_ASSIST_IDLE "204"
#define AIR_ASSIST_RUN  "1023"
#define EXHAUST_IDLE    "0"
#define EXHAUST_RUN     "65535"
#define INTAKE_IDLE     "0"
#define INTAKE_RUN      "43278"
/* Thermal-cooldown duty: half the run airflow - enough to keep pulling
 * heat out of the loop without the full-run roar. */
#define EXHAUST_COOL    "32768"
#define INTAKE_COOL     "21639"

/* Defaults, all env-adjustable (see gfcool_init). Temperature values are
 * the factory coolant-monitor windows (job-header CM*, millidegrees):
 * run ceiling CMrx=33 C, start/resume gate CMwx=31 C. */
#define HEATER_PCT_DEFAULT     10
#define COOLDOWN_SMOKE_S       15
#define COOLDOWN_MAX_S         300
#define TEMP_MAX_C_DEFAULT     33.0f
#define TEMP_RESUME_C_DEFAULT  31.0f

/* Flow-verification thresholds measured on this machine (file header). */
#define FLOW_FAULT_DT_C    4.5f
#define FLOW_REARM_DT_C    4.0f
#define FLOW_FAULT_HOLD_S  20
#define FLOW_ARM_DELAY_S   30

typedef enum {
    Cool_Idle = 0,
    Cool_Run,
    Cool_Smoke,      /* post-job smoke clear at run duty */
    Cool_Thermal,    /* reduced-duty airflow until temp recovers */
} cool_state_t;

static cool_state_t cool_state = Cool_Idle;
static coolant_state_t coolant_reported = {0};
static uint32_t heater_pct = HEATER_PCT_DEFAULT;
static uint32_t smoke_s = COOLDOWN_SMOKE_S;
static uint32_t cooldown_max_s = COOLDOWN_MAX_S;
static float temp_max_c = TEMP_MAX_C_DEFAULT;
static float temp_resume_c = TEMP_RESUME_C_DEFAULT;

static double phase_until;         /* smoke end / thermal timeout */
static double next_temp_check;
static double flow_armed_at;
static uint32_t flow_hot_secs = 0;
static bool flow_warned = false;
static bool temp_warned = false;
static bool hold_ours = false;
static bool forced_cool = false;   /* we overrode the phase fans to cool */

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

static void fans_cool (void)
{
    gfio_wr_attr("head/air_assist_pwm", AIR_ASSIST_IDLE);
    gfio_wr_attr("thermal/exhaust_pwm", EXHAUST_COOL);
    gfio_wr_attr("thermal/intake_pwm", INTAKE_COOL);
}

/* Reapply the fan profile the current phase calls for (used when an
 * over-temp intervention forced cooling airflow and then stood down). */
static void fans_apply_phase (void)
{
    switch(cool_state) {
        case Cool_Run:
        case Cool_Smoke:
            fans_run();
            break;
        case Cool_Thermal:
            fans_cool();
            break;
        default:
            fans_idle();
            break;
    }
}

static void heater_set (bool on)
{
    char val[16];
    snprintf(val, sizeof(val), "%u", on ? heater_pct * 65535 / 100 : 0);
    gfio_wr_attr("thermal/heater_pwm", val);
}

static bool read_temp (const char *attr, float *c)
{
    char buf[16];
    if(gfio_rd_attr(attr, buf, sizeof(buf)) != 0)
        return false;
    *c = (float)atoi(buf) * -0.09653f + 94.0f;
    return true;
}

static void warn (const char *msg)
{
    report_message(msg, Message_Warning);
    fprintf(stderr, "gfcool: %s\n", msg);
}

void gfcool_init (void)
{
    const char *opt;

    if((opt = getenv("GFCOOL_HEATER_PCT")))
        heater_pct = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_COOLDOWN_S")))
        smoke_s = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_COOLDOWN_MAX_S")))
        cooldown_max_s = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_TEMP_MAX")))
        temp_max_c = (float)atof(opt);
    if((opt = getenv("GFCOOL_TEMP_RESUME")))
        temp_resume_c = (float)atof(opt);

    gfio_wr_attr("thermal/water_pump_on", "1");
    gfio_wr_attr("thermal/tec_on", "0");
    gfio_wr_attr("head/purge_air", "1");
    heater_set(false);
    fans_idle();

    next_temp_check = wall_s() + 5.0;
}

void gfcool_coolant_set (coolant_state_t state)
{
    coolant_reported = state;

    if(state.flood) {
        cool_state = Cool_Run;
        fans_run();
        heater_set(true);
        flow_armed_at = wall_s() + FLOW_ARM_DELAY_S;
        flow_hot_secs = 0;
        flow_warned = false;
    } else if(cool_state == Cool_Run) {
        cool_state = Cool_Smoke;
        phase_until = wall_s() + (double)smoke_s;
        heater_set(false);          /* it would fight the thermal cooldown */
    }
}

coolant_state_t gfcool_coolant_get (void)
{
    return coolant_reported;
}

void gfcool_poll (void)
{
    double now = wall_s();

    if(now < next_temp_check)
        goto phases;
    next_temp_check = now + 1.0;

    float down, up;
    bool have_down = read_temp("pic/water_temp_1", &down);
    bool have_up = read_temp("pic/water_temp_2", &up);

    if(have_up) {
        sys_state_t st = state_get();

        /* Over-temp while executing: factory-style pause. grblHAL will
         * not enter HOLD from a jog (state_machine.c refuses it - jogs
         * are canceled, never held), so: cycles get a feed hold with
         * auto-resume; jogs get a jog-cancel (nothing to resume). Cooling
         * airflow is forced on either way. */
        if(up > temp_max_c && st == STATE_CYCLE && !hold_ours) {
            hold_ours = true;
            protocol_enqueue_realtime_command(CMD_FEED_HOLD);
            if(cool_state != Cool_Run) {
                fans_cool();
                forced_cool = true;
            }
            char msg[80];
            snprintf(msg, sizeof(msg),
                     "coolant %.1f C over %.0f C limit - pausing until %.0f C",
                     up, temp_max_c, temp_resume_c);
            warn(msg);
        } else if(up > temp_max_c && st == STATE_JOG) {
            protocol_enqueue_realtime_command(CMD_JOG_CANCEL);
            if(cool_state != Cool_Run) {
                fans_cool();
                forced_cool = true;
            }
            if(!temp_warned) {
                temp_warned = true;
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "coolant %.1f C over %.0f C limit - jog canceled", up, temp_max_c);
                warn(msg);
            }
        } else if(hold_ours) {
            if(!(st & STATE_HOLD)) {
                hold_ours = false;  /* operator resumed or reset; stand down */
                forced_cool = false;
                fans_apply_phase();
            } else if(up <= temp_resume_c) {
                hold_ours = false;
                forced_cool = false;
                warn("coolant temperature recovered - resuming");
                fans_apply_phase();
                protocol_enqueue_realtime_command(CMD_CYCLE_START);
            }
        } else if(up > temp_max_c && !temp_warned) {
            temp_warned = true;     /* not executing: warn only */
            char msg[64];
            snprintf(msg, sizeof(msg), "coolant temperature high: %.1f C", up);
            warn(msg);
        } else if(up < temp_max_c - 1.0f) {
            temp_warned = false;
            if(forced_cool) {       /* jog-cancel forced cooling; restore */
                forced_cool = false;
                fans_apply_phase();
            }
        }
    }

    /* Flow verification: only meaningful while the heater runs (Run
     * profile), after the delta has had time to establish. */
    if(cool_state == Cool_Run && heater_pct > 0 && now >= flow_armed_at
        && have_down && have_up) {
        float dt = down - up;
        if(dt > FLOW_FAULT_DT_C) {
            if(++flow_hot_secs >= FLOW_FAULT_HOLD_S && !flow_warned) {
                flow_warned = true;
                char msg[64];
                snprintf(msg, sizeof(msg), "coolant flow fault suspected (dT %.1f C)", dt);
                warn(msg);
            }
        } else if(dt < FLOW_REARM_DT_C) {
            flow_hot_secs = 0;
            flow_warned = false;
        }
    }

phases:
    if(cool_state == Cool_Smoke && now >= phase_until) {
        float up2;
        if(read_temp("pic/water_temp_2", &up2) && up2 > temp_resume_c) {
            cool_state = Cool_Thermal;
            phase_until = now + (double)cooldown_max_s;
            fans_cool();
        } else {
            cool_state = Cool_Idle;
            fans_idle();
        }
    } else if(cool_state == Cool_Thermal) {
        float up2;
        bool recovered = read_temp("pic/water_temp_2", &up2) && up2 <= temp_resume_c;
        if(recovered || now >= phase_until) {
            if(!recovered)
                warn("thermal cooldown timed out above the resume gate");
            cool_state = Cool_Idle;
            fans_idle();
        }
    }
}
