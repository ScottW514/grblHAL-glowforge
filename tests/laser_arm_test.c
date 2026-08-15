/*
  laser_arm_test.c - host unit test for the operator-arm coolant re-check

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later

  Regression for G-4: gflaser_arm() must re-check gfcool_fire_ok() after
  the operator button wait, immediately before it opens the armed
  window. The button wait can run for minutes, and the cached cooling
  verdict is refreshed under it (gfcool_poll, ~1 Hz); a verdict that
  went bad or stale during the wait must block the arm at the moment it
  matters, not only at the pre-wait check.

  The real function is exercised directly by including the driver source
  and driving gfcool_fire_ok() from a scripted sequence: the second
  reading models the verdict the button-wait window refreshed. With
  hw_active off (no GFSINK, as on the host) the two checks are the only
  gates in the path, so a scripted good-then-bad sequence isolates the
  post-wait re-check.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

/* gfcool_fire_ok() reads from this script, one entry per call. */
static bool fire_ok_script[4];
static int  fire_ok_n;
static int  fire_ok_calls;

static int   alarms_raised;
static char  last_message[128];
static bool  stream_armed;          /* gf_stream_laser_arm(true) reached? */
static bool  latch_locked_last;

bool gfcool_fire_ok(void)
{
    int i = fire_ok_calls < fire_ok_n ? fire_ok_calls : fire_ok_n - 1;
    fire_ok_calls++;
    return fire_ok_script[i];
}

void gfcool_laser_armed(bool armed) { (void)armed; }
void gf_stream_laser(unsigned char power, bool fire) { (void)power; (void)fire; }
void gf_stream_laser_arm(bool armed) { stream_armed = armed; }
void gf_stream_laser_latch(bool lock) { latch_locked_last = lock; }
int  gfio_rd_attr(const char *a, char *b, size_t l)
{ (void)a; if (l) b[0] = '\0'; return -1; }
float gfio_conf_read_float(const char *k, float fb) { (void)k; return fb; }
void serial_poll(void) {}
void serial_wait(long us) { (void)us; }
unsigned serial_client_generation(void) { return 1; }
bool protocol_execute_realtime(void) { return true; }

/* --- driver source under test ---------------------------------------- */
#include "../src/glowforge_laser.c"

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
settings_t settings;
grbl_t grbl;

void report_message(const char *msg, message_type_t type)
{
    (void)type;
    snprintf(last_message, sizeof(last_message), "%s", msg ? msg : "");
}
void system_raise_alarm(alarm_code_t alarm) { (void)alarm; alarms_raised++; }
sys_state_t state_get(void) { return STATE_IDLE; }
bool spindle_precompute_pwm_values(spindle_ptrs_t *s, spindle_pwm_t *p,
                                   spindle_pwm_settings_t *cfg, uint32_t hz)
{ (void)s; (void)p; (void)cfg; (void)hz; return true; }
spindle_id_t spindle_register(const spindle_ptrs_t *s, const char *n)
{ (void)s; (void)n; return 0; }

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static void reset_state(void)
{
    fire_ok_calls = 0;
    alarms_raised = 0;
    last_message[0] = '\0';
    stream_armed = false;
    latch_locked_last = true;
    laser_ok = false;
    disarm_request = false;
    hw_active = false;              /* host: no GFSINK, no button device */
}

static void script(bool a, bool b, int n)
{
    fire_ok_script[0] = a;
    fire_ok_script[1] = b;
    fire_ok_n = n;
}

int main(void)
{
    printf("gflaser_arm() coolant re-check (G-4):\n");

    /* Case A - the G-4 case: the gate is clear at the pre-wait check but
       has gone bad by the post-wait re-check. Arming must refuse. */
    reset_state();
    script(true, false, 2);
    bool armed_a = gflaser_arm();
    CHECK(!armed_a, "refuses when the verdict goes bad during the wait");
    CHECK(!laser_ok, "armed window stays closed on the late refusal");
    CHECK(!stream_armed, "stream is never told armed on the late refusal");
    CHECK(latch_locked_last, "latch is left locked on the late refusal");
    CHECK(alarms_raised == 1, "raises an alarm on the late refusal");
    CHECK(fire_ok_calls == 2, "the coolant gate is re-checked after the wait");
    CHECK(strstr(last_message, "blocked") != NULL,
          "reports the fire-blocked reason");

    /* Case B - clear at both checks: arming succeeds. */
    reset_state();
    script(true, true, 2);
    bool armed_b = gflaser_arm();
    CHECK(armed_b, "arms when the gate is clear at both checks");
    CHECK(laser_ok, "armed window opens when clear");
    CHECK(stream_armed, "stream is told armed when clear");

    /* Case C - blocked at the pre-wait check: refuses before arming. */
    reset_state();
    script(false, false, 2);
    bool armed_c = gflaser_arm();
    CHECK(!armed_c, "refuses when the gate is blocked up front");
    CHECK(!laser_ok, "armed window stays closed on the early refusal");
    CHECK(fire_ok_calls == 1, "the pre-wait check short-circuits the arm");

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the arm re-checks the coolant gate after the wait\n",
           failures);
    return failures ? 1 : 0;
}
