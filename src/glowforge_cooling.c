/*
  glowforge_cooling.c - fan / coolant management for the Glowforge board

  Part of grblHAL-glowforge. Job-state fan and coolant policy built from
  the factory's own numbers (captured pulse-file headers + settings map):
  fan duties AAid=204/AArd=1023, EFrd=65535, IFrd=43278; coolant windows
  CM* in millidegrees - run ceiling 33 C (CMrx), start/resume gate 31 C
  (CMwx), idle max 50 C. All thresholds env-adjustable.

  Policy:
  - IDLE: pump on, TEC off, purge on, fans at factory idle, HEATER OFF
    (an always-on flow heater measurably warms the loop - ~1.5 C in
    minutes on this machine - eating headroom below the 31 C start gate
    for no benefit while nothing can fire).
  - RUN (M8, LightBurn's per-layer air assist): cut-profile fans, plus
    periodic coolant-flow verification. The flow heater sits between
    the two water-temp sensors; each check runs it at FLOW_HEATER_PCT
    for FLOW_CHECK_S and reads how far the downstream sensor climbs
    (flowing coolant carries the heat away; a stagnant loop cooks the
    downstream sensor). Checks repeat every FLOW_RECHECK_S and start
    only from a thermally settled loop; the constants below carry the
    measured rationale. An over-limit reading is a SUSPICION, not a
    fault: it warns and requests an immediate re-check, and only a
    second consecutive over-limit (no clean check in between,
    whatever the wall-clock gap) - or a suspicion the loop cannot
    resolve within FLOW_CONFIRM_MAX_S - raises COOLANT FLOW FAULT.
    A clean re-check clears the suspicion: transient events (a pump
    airlock burp, disturbed coolant) self-clear within minutes, while
    true stagnation cannot pass a settled re-check. Cleared
    suspicions still count - FLOW_TREND_N of them in one job earn an
    aggregated check-your-coolant warning.
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
  from warnings/holds into hard fire interlocks; a flow suspicion will
  then also take the safe posture (hold, laser off, forced cooling)
  while its re-check decides.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <math.h>
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
#define COOLDOWN_SMOKE_S       15
#define COOLDOWN_MAX_S         300
#define TEMP_MAX_C_DEFAULT     33.0f
#define TEMP_RESUME_C_DEFAULT  31.0f

/* Flow check. Characterized on the machine with the factory temperature
 * curve (scripts/bench/flow_characterize.py):
 *   heater 10% -> flow dT <= +3.69, no-flow dT >= +3.74  (0.04 C gap:
 *                 UNUSABLE, sensor noise alone spans ~0.9 C)
 *   heater 30% -> flow dT <= +9.32, no-flow dT >= +10.99 (1.67 C gap)
 * So the check runs at 30% and only briefly: the delta plateaus by
 * ~30 s, and continuous 30% heating would add ~0.7 C/min to the loop.
 * After the check the heater goes off and absolute-temperature
 * monitoring carries the protection (a pump failure mid-cut shows up
 * far faster as a temperature climb than as a heater delta). */
/* Duty matters more than anything else here. Below ~40% the stagnant
 * loop sheds the heater's output by natural convection well enough to
 * MIMIC FLOW: at 30%/50 s the five no-flow trials read 8.15, 8.69,
 * 8.78, 12.25, 13.33 C while flow never exceeded 9.08 - three of five
 * dead-pump cases look healthier than a working pump. At 40% the heat
 * input outruns convection and the signature becomes decisive and
 * repeatable (see the design matrix, scripts/bench/flow_matrix.py). */
#define FLOW_HEATER_PCT    40
#define FLOW_CHECK_S       50      /* 0 disables the check entirely */
#define FLOW_ESTABLISH_S   30      /* delta plateaus by here (reporting only) */
/* Re-check cadence while a job runs. A stopped pump CANNOT be seen any
 * other way: absolute temperature only tracks a circulating loop, and a
 * light engrave may add so little heat that a flat trend proves
 * nothing. Detection latency is this interval plus the check length. */
#define FLOW_RECHECK_S     150

/* A check is only meaningful from a thermally SETTLED loop. If the
 * downstream sensor is still falling from earlier heating, the baseline
 * is captured mid-transient and the measured rise is garbage -
 * bench-proven to misclassify in BOTH directions, including reporting
 * flow when the pump was stopped (a MISS).
 *
 * Sensor agreement alone is NOT sufficient: both sensors can be
 * plunging together and cross within any tolerance while the loop is
 * still far from steady - that exact case produced a miss on the bench
 * (rise 11.2 with the pump stopped, from a loop cooling out of 48 C).
 * So the gate also requires the downstream reading to be STATIONARY
 * over a window before the baseline is taken.
 *
 * Stationarity is measured as the difference between the mean of the
 * newer half of the window and the mean of the older half. Peak-to-peak
 * does not work: measured on a settled loop this sensor shows 0.52 C of
 * p-p jitter over 15 s (0.70 worst), so any p-p threshold tight enough
 * to catch drift sits below the noise floor and the gate never opens.
 * Averaging each
 * half cuts that to 0.11 C typical / 0.21 C worst, while a real cooling
 * transient (~2 C per 15 s) still shows ~1.5 C. */
#define FLOW_SETTLE_DT_C    1.5f    /* |downstream - upstream| */
#define FLOW_SETTLE_DRIFT_C 0.4f    /* split-half mean difference */
#define FLOW_SETTLE_WIN     15      /* 1 Hz samples */
#define FLOW_SETTLE_WARN_S  180

/* The DISCRIMINATOR is how far the downstream sensor climbs during the
 * check, not the upstream/downstream delta. Measured from cold at 30%
 * heater: with flow the downstream plateaus at about +10.5 C, without
 * flow it passes +16 C and is still climbing - a ~6 C margin, versus
 * only ~2 C for the delta. The delta is NOT a usable discriminator: a
 * check that starts from a cold heater never reaches the steady-state
 * delta (bench: a live pump-off drill reads 8.8 C against a 10.2 C
 * limit - a false negative). */
/* Balanced midpoint of the pooled bracket at 40%/50 s: across 17 flow
 * observations (design matrix, repeat validations, and a 40-minute
 * sustained run) the largest healthy rise was 12.75 C, and across 8
 * pump-stopped observations the smallest was 16.04 C. 14.4 sits ~1.6 C
 * from either edge. All baselines were 19-23 C; the warm-loop end of
 * the range is NOT yet validated (a first-light commissioning item).
 * GFCOOL_FLOW_RISE overrides. */
#define FLOW_FAULT_RISE_C  14.4f

/* A suspicion must resolve. With flow, the check's own heat sheds in
 * under a minute and the confirming verdict lands 2-4 min after the
 * suspect one; with a truly dead pump the cooked region needs ~3-5 min
 * of conduction-only decay before the settle gate reopens, and the
 * re-check then reads stagnant again. A loop that cannot produce any
 * verdict inside this budget has shown no evidence of health and is
 * treated as faulted. The budget restarts with each flood session -
 * checks cannot run outside one. GFCOOL_CONFIRM_MAX_S overrides. */
#define FLOW_CONFIRM_MAX_S 480
/* Cleared suspicions per job that earn an aggregated warning: each one
 * was disproven by its re-check, but several in a single job point at
 * a marginal pump or recurring airlock. */
#define FLOW_TREND_N       3

typedef enum {
    Cool_Idle = 0,
    Cool_Run,
    Cool_Smoke,      /* post-job smoke clear at run duty */
    Cool_Thermal,    /* reduced-duty airflow until temp recovers */
} cool_state_t;

static cool_state_t cool_state = Cool_Idle;
static coolant_state_t coolant_reported = {0};
static uint32_t smoke_s = COOLDOWN_SMOKE_S;
static uint32_t cooldown_max_s = COOLDOWN_MAX_S;
static float temp_max_c = TEMP_MAX_C_DEFAULT;
static float temp_resume_c = TEMP_RESUME_C_DEFAULT;
static float flow_fault_rise = FLOW_FAULT_RISE_C;
static uint32_t flow_heater_pct = FLOW_HEATER_PCT;
static uint32_t flow_check_s = FLOW_CHECK_S;
static uint32_t flow_recheck_s = FLOW_RECHECK_S;
static double flow_next_check;
static bool flow_check_active = false;
static bool flow_check_pending = false;
static double flow_pending_since;
static bool flow_settle_warned = false;
static bool flow_base_set = false;
static float down_hist[FLOW_SETTLE_WIN];
static uint32_t down_hist_n = 0;
static float flow_base_down;
static double flow_establish_at, flow_check_end;
static float flow_dt_sum;
static uint32_t flow_dt_n;

/* One over-limit reading opens a suspicion; the next completed check
 * decides it, whenever that is - "consecutive" means no clean check in
 * between, not close in time. */
typedef enum {
    Flow_Normal = 0,
    Flow_Suspect,       /* one over-limit; the re-check decides */
    Flow_Fault,         /* consecutive over-limits, or a starved re-check */
} flow_verdict_t;

static flow_verdict_t flow_verdict = Flow_Normal;
static double flow_suspect_since;
static uint32_t confirm_max_s = FLOW_CONFIRM_MAX_S;
static uint32_t flow_episodes = 0;  /* cleared suspicions this job */

static double phase_until;         /* smoke end / thermal timeout */
static double next_temp_check;
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

static void heater_set_pct (uint32_t pct)
{
    char val[16];
    snprintf(val, sizeof(val), "%u", pct * 65535 / 100);
    gfio_wr_attr("thermal/heater_pwm", val);
}

/* Begin a flow check: heater up, baseline captured on the next poll,
 * verdict at the end of the window, heater back off. */
static void flow_check_start (double now)
{
    flow_check_active = true;
    flow_base_set = false;
    flow_establish_at = now + FLOW_ESTABLISH_S;
    flow_check_end = now + (double)flow_check_s;
    flow_dt_sum = 0.0f;
    flow_dt_n = 0;
    heater_set_pct(flow_heater_pct);
}

/* Raw ADC -> degrees C for the two coolant thermistors.
 *
 * This is the FACTORY conversion, recovered from the v2.6.0 firmware
 * binary (forward function at 0x65110, inverse at 0x64ee8, parameter
 * block in .data at VMA 0x11e120): a single-parameter B-equation NTC
 * behind a divider and gain stage, i.e.
 *
 *   R     = Rd / (F/raw - 1),  F = adc_steps * gain
 *   T[K]  = beta / ln(R / Rinf),  Rinf = R0 * exp(-beta/T0)
 *
 * with R0 = 10k at T0 = 298.15 K, beta = 3380, Rd = 10k, gain = 1.3,
 * adc_steps = 1024. Proof it is the right one: running the firmware's
 * inverse on the cloud's coolant setpoints reproduces this machine's
 * WT* settings exactly (CMet 18134 mDeg -> raw 754 = WTub; CMdt 18364
 * -> raw 751 = WTvb); measured against a thermometer at equilibrium
 * the curve lands within ~1 C. */
static float thermistor_c (int raw)
{
    static const double adc_f = 1024.0 * 1.3;   /* 1331.2 */
    static const double rd = 10000.0;
    static const double beta = 3380.0;
    double rinf = 10000.0 * exp(-3380.0 / 298.15);

    if(raw <= 0 || (double)raw >= adc_f)
        return -273.15f;                        /* open / shorted */

    double r = rd / (adc_f / (double)raw - 1.0);

    return (float)(beta / log(r / rinf) - 273.15);
}

static bool read_temp (const char *attr, float *c)
{
    char buf[16];
    if(gfio_rd_attr(attr, buf, sizeof(buf)) != 0)
        return false;
    *c = thermistor_c(atoi(buf));
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

    if((opt = getenv("GFCOOL_FLOW_HEATER_PCT")))
        flow_heater_pct = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_FLOW_CHECK_S")))
        flow_check_s = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_RECHECK_S")))
        flow_recheck_s = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_COOLDOWN_S")))
        smoke_s = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_COOLDOWN_MAX_S")))
        cooldown_max_s = (uint32_t)atoi(opt);
    if((opt = getenv("GFCOOL_TEMP_MAX")))
        temp_max_c = (float)atof(opt);
    if((opt = getenv("GFCOOL_TEMP_RESUME")))
        temp_resume_c = (float)atof(opt);
    if((opt = getenv("GFCOOL_FLOW_RISE")))
        flow_fault_rise = (float)atof(opt);
    if((opt = getenv("GFCOOL_CONFIRM_MAX_S")))
        confirm_max_s = (uint32_t)atoi(opt);

    gfio_wr_attr("thermal/water_pump_on", "1");
    gfio_wr_attr("thermal/tec_on", "0");
    gfio_wr_attr("head/purge_air", "1");
    heater_set_pct(0);
    fans_idle();

    next_temp_check = wall_s() + 5.0;
}

void gfcool_coolant_set (coolant_state_t state)
{
    coolant_reported = state;

    if(state.flood) {
        cool_state = Cool_Run;
        fans_run();
        if(flow_verdict == Flow_Suspect)
            flow_suspect_since = wall_s();  /* budget is per flood session */
        /* Request a check at job start; the poll starts it once the
         * loop is settled, then repeats it on the re-check cadence. */
        if(flow_check_s > 0 && !flow_check_active && !flow_check_pending) {
            flow_check_pending = true;
            flow_pending_since = wall_s();
            flow_settle_warned = false;
        }
    } else if(cool_state == Cool_Run) {
        cool_state = Cool_Smoke;
        phase_until = wall_s() + (double)smoke_s;
        flow_check_pending = false;
        if(flow_check_active) {     /* job ended before the verdict */
            flow_check_active = false;
            heater_set_pct(0);
        }
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

    /* One-shot flow check: how far does the downstream sensor climb
     * while the heater runs? Flow carries the heat away (plateau);
     * no flow lets it pile up. The delta is averaged too, but only for
     * the report - it is the weaker discriminator (see file header). */
    if(flow_check_active && have_down && have_up) {
        if(!flow_base_set) {
            flow_base_down = down;
            flow_base_set = true;
        }
        if(now >= flow_establish_at) {
            flow_dt_sum += down - up;
            flow_dt_n++;
        }
        if(now >= flow_check_end) {
            float rise = down - flow_base_down;
            float dt = flow_dt_n ? flow_dt_sum / (float)flow_dt_n : 0.0f;
            char msg[112];

            flow_check_active = false;
            heater_set_pct(0);
            /* Schedule the next interrogation if the job is still on. */
            flow_next_check = now + (double)flow_recheck_s;

            if(rise > flow_fault_rise) {
                if(flow_verdict == Flow_Normal) {
                    /* First over-limit: open a suspicion and re-check
                     * as soon as the loop settles instead of waiting
                     * out the cadence. */
                    flow_verdict = Flow_Suspect;
                    flow_suspect_since = now;
                    flow_check_pending = true;
                    flow_pending_since = now;
                    flow_settle_warned = false;
                    snprintf(msg, sizeof(msg),
                             "COOLANT FLOW SUSPECT: heater rise %.1f C (limit %.1f, dT %.1f) - re-checking",
                             rise, flow_fault_rise, dt);
                } else {
                    flow_verdict = Flow_Fault;
                    snprintf(msg, sizeof(msg),
                             "COOLANT FLOW FAULT: heater rise %.1f C (limit %.1f, dT %.1f) - check the pump",
                             rise, flow_fault_rise, dt);
                }
                warn(msg);
            } else if(flow_verdict != Flow_Normal) {
                snprintf(msg, sizeof(msg),
                         flow_verdict == Flow_Suspect
                           ? "coolant flow suspicion cleared (heater rise %.1f C, dT %.1f C)"
                           : "coolant flow recovered (heater rise %.1f C, dT %.1f C)",
                         rise, dt);
                flow_verdict = Flow_Normal;
                report_message(msg, Message_Info);
                fprintf(stderr, "gfcool: %s\n", msg);
                if(++flow_episodes == FLOW_TREND_N) {
                    snprintf(msg, sizeof(msg),
                             "%u coolant flow suspicions this job - check the pump and coolant level",
                             (unsigned)flow_episodes);
                    warn(msg);
                }
            } else {
                snprintf(msg, sizeof(msg),
                         "coolant flow verified (heater rise %.1f C, dT %.1f C)", rise, dt);
                report_message(msg, Message_Info);
                fprintf(stderr, "gfcool: %s\n", msg);
            }
        }
    } else if(cool_state == Cool_Run && flow_check_s > 0 && flow_recheck_s > 0
               && !flow_check_pending && now >= flow_next_check) {
        flow_check_pending = true;  /* periodic re-interrogation mid-job */
        flow_pending_since = now;
        flow_settle_warned = false;
    }

    /* A suspicion may not sit unresolved while a job runs: no verdict
     * within the budget (settle gate starved - itself consistent with
     * stagnation, which decays by conduction only) fails safe. An
     * in-flight check is left to deliver its real verdict instead. */
    if(flow_verdict == Flow_Suspect && !flow_check_active
       && cool_state == Cool_Run
       && now - flow_suspect_since > (double)confirm_max_s) {
        char msg[96];
        flow_verdict = Flow_Fault;
        snprintf(msg, sizeof(msg),
                 "COOLANT FLOW FAULT: no clean re-check within %u s - check the pump",
                 (unsigned)confirm_max_s);
        warn(msg);
    }

    /* Track downstream history for the stationarity test below. It is
     * only meaningful while the heater is off, so a running check
     * invalidates it. */
    if(have_down) {
        if(flow_check_active)
            down_hist_n = 0;
        else {
            down_hist[down_hist_n % FLOW_SETTLE_WIN] = down;
            down_hist_n++;
        }
    }

    /* Gate: a pending check only starts from a settled loop - sensors
     * in agreement AND the downstream reading stationary. */
    if(flow_check_pending && !flow_check_active && have_down && have_up) {
        bool stationary = down_hist_n >= FLOW_SETTLE_WIN;
        if(stationary) {
            uint32_t base = down_hist_n - FLOW_SETTLE_WIN;
            float old_sum = 0.0f, new_sum = 0.0f;
            for(uint32_t i = 0; i < 7; i++)
                old_sum += down_hist[(base + i) % FLOW_SETTLE_WIN];
            for(uint32_t i = 7; i < FLOW_SETTLE_WIN; i++)
                new_sum += down_hist[(base + i) % FLOW_SETTLE_WIN];
            stationary = fabsf(new_sum / 8.0f - old_sum / 7.0f) <= FLOW_SETTLE_DRIFT_C;
        }
        if(stationary && fabsf(down - up) <= FLOW_SETTLE_DT_C) {
            flow_check_pending = false;
            flow_check_start(now);
        } else if(!flow_settle_warned && now - flow_pending_since > FLOW_SETTLE_WARN_S) {
            flow_settle_warned = true;
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "flow check deferred: loop not settled (dT %.1f C, %s)",
                     down - up, stationary ? "drifting" : "sensors disagree");
            warn(msg);
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
            flow_episodes = 0;
        }
    } else if(cool_state == Cool_Thermal) {
        float up2;
        bool recovered = read_temp("pic/water_temp_2", &up2) && up2 <= temp_resume_c;
        if(recovered || now >= phase_until) {
            if(!recovered)
                warn("thermal cooldown timed out above the resume gate");
            cool_state = Cool_Idle;
            fans_idle();
            flow_episodes = 0;
        }
    }
}
