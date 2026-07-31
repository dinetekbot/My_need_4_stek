/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_control.c -- steering law, throttle law, driving state machine.
*/

#include <stdio.h>
#include <string.h>

#include "control.h"
#include "test.h"

static void plain_config(config_t *cfg)
{
    config_defaults(cfg);
    cfg->smooth_window = 1;
}

static void build_scan(scan_t *scan, const config_t *cfg, double value)
{
    double raw[N4S_LIDAR_RAYS];

    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        raw[i] = value;
    scan_build(scan, raw, cfg);
}

static void test_heading_error(void)
{
    gap_t gap;

    memset(&gap, 0, sizeof(gap));
    gap.found = true;
    gap.target_angle = 0.0;
    CHECK_NEAR(heading_error(&gap), 0.0, 1e-9);
    /* The edge of the cone maps to nearly full deflection. */
    gap.target_angle = 30.0;
    CHECK_NEAR(heading_error(&gap), 1.0, 1e-9);
    gap.target_angle = -15.0;
    CHECK_NEAR(heading_error(&gap), -0.5, 1e-9);
    /* No gap means no opinion, not a hard turn in some arbitrary direction. */
    gap.found = false;
    CHECK_NEAR(heading_error(&gap), 0.0, 1e-9);
    CHECK_NEAR(heading_error(NULL), 0.0, 1e-9);
}

static void test_centring_error(void)
{
    scan_t scan;

    memset(&scan, 0, sizeof(scan));
    scan.left_mm = 1000.0;
    scan.right_mm = 1000.0;
    CHECK_NEAR(centring_error(&scan), 0.0, 1e-9);
    /* More room on the right: the car is hugging the left wall and the term
    ** should push it back to the middle. */
    scan.left_mm = 500.0;
    scan.right_mm = 1500.0;
    CHECK_NEAR(centring_error(&scan), 0.5, 1e-9);
    scan.left_mm = 1500.0;
    scan.right_mm = 500.0;
    CHECK_NEAR(centring_error(&scan), -0.5, 1e-9);
    /* A scan of zeros must not divide by zero. */
    scan.left_mm = 0.0;
    scan.right_mm = 0.0;
    CHECK_NEAR(centring_error(&scan), 0.0, 1e-9);
}

/* The actuator must never be asked to jump. A full-lock command out of
** nowhere unsettles the car and, in the physics engine, spins it. */
static void test_steering_is_rate_limited(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    double wheels;

    plain_config(&cfg);
    cfg.steer_balance = 0.0;
    controller_init(&ctl);
    build_scan(&scan, &cfg, 2000.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = true;
    gap.target_angle = 30.0;

    wheels = steering_command(&ctl, &scan, &gap, &cfg);
    CHECK_NEAR(wheels, cfg.steer_rate_limit, 1e-9);
    wheels = steering_command(&ctl, &scan, &gap, &cfg);
    CHECK_NEAR(wheels, 2.0 * cfg.steer_rate_limit, 1e-9);
}

static void test_steering_is_clamped(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    double wheels = 0.0;

    plain_config(&cfg);
    cfg.steer_kp = 5.0;          /* deliberately absurd */
    cfg.steer_rate_limit = 10.0; /* let it reach the clamp in one step */
    controller_init(&ctl);
    build_scan(&scan, &cfg, 2000.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = true;
    gap.target_angle = 30.0;
    for (int i = 0; i < 5; i++)
        wheels = steering_command(&ctl, &scan, &gap, &cfg);
    /* Whatever the gains, WHEELS_DIR leaves this function inside [-1;1].
    ** Out-of-range values are rejected by the API, so an unclamped controller
    ** does not steer harder -- it stops steering at all. */
    CHECK(wheels <= 1.0 && wheels >= -1.0);
    CHECK_NEAR(wheels, 1.0, 1e-9);
}

/* Aiming left must steer left, on both sides, with no dead zone at zero. */
static void test_steering_sign(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;

    plain_config(&cfg);
    cfg.steer_balance = 0.0;
    build_scan(&scan, &cfg, 2000.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = true;

    controller_init(&ctl);
    gap.target_angle = -20.0;
    CHECK(steering_command(&ctl, &scan, &gap, &cfg) < 0.0);

    controller_init(&ctl);
    gap.target_angle = 20.0;
    CHECK(steering_command(&ctl, &scan, &gap, &cfg) > 0.0);

    controller_init(&ctl);
    gap.target_angle = 0.0;
    CHECK_NEAR(steering_command(&ctl, &scan, &gap, &cfg), 0.0, 1e-9);
}

static void test_throttle_profile(void)
{
    config_t cfg;
    scan_t scan;

    plain_config(&cfg);
    /* Clear road ahead, wheels straight: full power. */
    build_scan(&scan, &cfg, 3000.0);
    CHECK_NEAR(throttle_command(&scan, 0.0, &cfg), cfg.throttle_max, 1e-9);
    /* Same road, wheels hard over: give up turn_penalty of it. */
    CHECK_NEAR(throttle_command(&scan, 1.0, &cfg),
        cfg.throttle_max * (1.0 - cfg.turn_penalty), 1e-9);
    /* The penalty depends on how hard we steer, not which way. */
    CHECK_NEAR(throttle_command(&scan, -1.0, &cfg),
        throttle_command(&scan, 1.0, &cfg), 1e-9);
    /* Close to a wall: creep. */
    build_scan(&scan, &cfg, cfg.dist_slow_mm);
    CHECK_NEAR(throttle_command(&scan, 0.0, &cfg), cfg.throttle_min, 1e-9);
    /* Too close: nothing at all. */
    build_scan(&scan, &cfg, cfg.brake_dist_mm - 1.0);
    CHECK_NEAR(throttle_command(&scan, 0.0, &cfg), 0.0, 1e-9);
}

/* Whatever the inputs, the two actuator commands stay inside their ranges. */
static void test_throttle_stays_in_range(void)
{
    config_t cfg;
    scan_t scan;

    plain_config(&cfg);
    for (double distance = 0.0; distance <= N4S_LIDAR_MAX_MM; distance += 37.0) {
        build_scan(&scan, &cfg, distance);
        for (double wheels = -1.0; wheels <= 1.0; wheels += 0.25) {
            double power = throttle_command(&scan, wheels, &cfg);

            CHECK(power >= 0.0 && power <= 1.0);
        }
    }
}

/* One bad scan is noise. Enough of them in a row is a wall. */
static void test_deadend_needs_persistence(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    /* Far enough not to trigger the reverse rule, blank enough to find no gap. */
    build_scan(&scan, &cfg, cfg.reverse_dist_mm * 3.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = false;

    for (int i = 0; i < cfg.deadend_cycles - 1; i++)
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_RACING);
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_HALTING);
}

/* A single gapless cycle in the middle of a run must not end the race. */
static void test_single_blind_cycle_is_ignored(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t open_gap;
    gap_t blind_gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    build_scan(&scan, &cfg, 2500.0);
    memset(&open_gap, 0, sizeof(open_gap));
    open_gap.found = true;
    open_gap.depth_mm = 2500.0;
    memset(&blind_gap, 0, sizeof(blind_gap));

    for (int i = 0; i < cfg.deadend_cycles * 3; i++)
        controller_step(&ctl, &scan, (i % 5 == 0) ? &blind_gap : &open_gap,
            &cfg, &cmd);
    CHECK(ctl.state == DRIVE_RACING);
}

static void test_reversing_when_too_close(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    memset(&gap, 0, sizeof(gap));
    gap.found = true;

    build_scan(&scan, &cfg, cfg.reverse_dist_mm - 10.0);
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_REVERSING);

    /* While reversing: backwards power, no forward power. */
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK_NEAR(cmd.throttle, 0.0, 1e-9);
    CHECK_NEAR(cmd.reverse, cfg.reverse_power, 1e-9);

    /* Backing out until there is clearly room again, with hysteresis so the
    ** car does not shuffle back and forth on the threshold. */
    build_scan(&scan, &cfg, cfg.reverse_dist_mm * 1.2);
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_REVERSING);
    build_scan(&scan, &cfg, cfg.reverse_dist_mm * 3.0);
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_RACING);
}

/*
** The rules require the car to come to rest at least three feet from the wall
** before ending the simulation in a dead end. Halting with the bumper against
** the wall is a failed run even though the car did stop.
*/
static void test_halting_backs_off_to_legal_clearance(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    ctl.state = DRIVE_HALTING;
    memset(&gap, 0, sizeof(gap));

    /* Nose too close to the wall: keep backing up, never declare finished. */
    build_scan(&scan, &cfg, cfg.deadend_clearance_mm - 200.0);
    for (int i = 0; i < 20; i++) {
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
        CHECK(ctl.state == DRIVE_HALTING);
        CHECK(cmd.reverse > 0.0);
    }
    /* Enough clearance: settle, then finish. */
    build_scan(&scan, &cfg, cfg.deadend_clearance_mm + 200.0);
    for (int i = 0; i < 20; i++)
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_FINISHED);
    CHECK_NEAR(cmd.throttle, 0.0, 1e-9);
    CHECK_NEAR(cmd.reverse, 0.0, 1e-9);
}

void test_control(void)
{
    printf("control\n");
    test_heading_error();
    test_centring_error();
    test_steering_is_rate_limited();
    test_steering_is_clamped();
    test_steering_sign();
    test_throttle_profile();
    test_throttle_stays_in_range();
    test_deadend_needs_persistence();
    test_single_blind_cycle_is_ignored();
    test_reversing_when_too_close();
    test_halting_backs_off_to_legal_clearance();
}
