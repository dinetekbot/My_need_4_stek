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
    /* Pin the ray order rather than inheriting the platform default: these
    ** tests are about geometry, not about which way round the real sensor
    ** happens to be wired. */
    cfg->mirror = false;
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
    /* The thresholds have to be ordered for any of this to mean anything:
    ** the slow end of the ramp must sit at or above the braking distance,
    ** which must sit above the distance at which we give up and reverse.
    ** Defaults that violate this leave whole bands of behaviour unreachable. */
    CHECK(cfg.dist_slow_mm >= cfg.brake_dist_mm);
    CHECK(cfg.brake_dist_mm > cfg.reverse_dist_mm);
    CHECK(cfg.dist_fast_mm > cfg.dist_slow_mm);
    /* Close to a wall: creep. */
    build_scan(&scan, &cfg, cfg.dist_slow_mm);
    CHECK_NEAR(throttle_command(&scan, 0.0, &cfg), cfg.throttle_min, 1e-9);
    /* Below the braking distance the throttle law itself yields nothing; it
    ** is the state machine that decides between crawling and reversing. */
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

/*
** Never drive into space the gap search could not find.
**
** An earlier version computed the throttle from the forward distance alone,
** so with no gap it happily crept forward at 0.15 power. Observed on a real
** track: four cycles of creeping straight into a wall it had already failed
** to find a way around.
*/
static void test_never_drives_without_a_gap(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    /* Wide open ahead by the distance metric -- the only thing missing is a
    ** gap. The throttle must still be zero. */
    build_scan(&scan, &cfg, 3000.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = false;
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK_NEAR(cmd.throttle, 0.0, 1e-9);
}

/*
** The dead-zone regression.
**
** With separate brake and reverse thresholds there is a band -- closer than
** brake_dist, further than reverse_dist -- where the old state machine cut
** the throttle but did not start reversing. The car parked itself there and
** waited. It must instead notice it is making no progress and recover.
*/
static void test_no_dead_zone_between_brake_and_reverse(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;
    double middle;

    plain_config(&cfg);
    controller_init(&ctl);
    middle = (cfg.brake_dist_mm + cfg.reverse_dist_mm) / 2.0;
    CHECK(middle < cfg.brake_dist_mm && middle > cfg.reverse_dist_mm);
    build_scan(&scan, &cfg, middle);
    /* A gap is visible: this is not a blindness problem, it is a "too close
    ** to use power, not close enough to reverse" problem. */
    memset(&gap, 0, sizeof(gap));
    gap.found = true;
    gap.target_angle = 20.0;

    /* The invariant: the car is never idle. Every cycle it either crawls
    ** forward towards the gap or backs away -- never nothing. */
    for (int i = 0; i < cfg.blind_cycles * 4; i++) {
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
        CHECK(cmd.throttle > 0.0 || cmd.reverse > 0.0);
    }
    /* With a way out in sight, the right answer is to crawl round it, not to
    ** give up and reverse. */
    CHECK(ctl.state == DRIVE_RACING);
    CHECK_NEAR(cmd.throttle, cfg.crawl_power, 1e-9);
}

/* No gap and too close to power out of it: back away, do not sit there. */
static void test_stuck_without_a_gap_reverses(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    build_scan(&scan, &cfg, (cfg.brake_dist_mm + cfg.reverse_dist_mm) / 2.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = false;

    for (int i = 0; i < cfg.blind_cycles; i++)
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_REVERSING);
}

/* One gapless cycle in the middle of a run is noise, not a dead end. */
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

    for (int i = 0; i < 60; i++)
        controller_step(&ctl, &scan,
            (i % (cfg.blind_cycles + 2) == 0) ? &blind_gap : &open_gap,
            &cfg, &cmd);
    CHECK(ctl.state == DRIVE_RACING);
}

/* A genuine dead end: repeated recoveries that never lead anywhere. */
static void test_repeated_failed_recoveries_end_the_race(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    build_scan(&scan, &cfg, cfg.reverse_dist_mm / 2.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = false;

    for (int i = 0; i < cfg.deadend_attempts * cfg.max_reverse_cycles * 2; i++) {
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
        if (ctl.state == DRIVE_HALTING || ctl.state == DRIVE_FINISHED)
            break;
    }
    CHECK(ctl.state == DRIVE_HALTING || ctl.state == DRIVE_FINISHED);
}

/* Getting properly going again must forgive the earlier recoveries, or a
** track with several tight corners eventually reads as a dead end. */
static void test_clean_racing_clears_the_recovery_count(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;

    plain_config(&cfg);
    controller_init(&ctl);
    ctl.recoveries = cfg.deadend_attempts - 1;
    build_scan(&scan, &cfg, 2800.0);
    memset(&gap, 0, sizeof(gap));
    gap.found = true;
    gap.depth_mm = 2800.0;

    for (int i = 0; i < cfg.recovery_reset_cycles + 2; i++)
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.recoveries == 0);
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
    /* Enough clearance, with the margin: settle, then finish. */
    build_scan(&scan, &cfg, cfg.deadend_clearance_mm * 1.4);
    for (int i = 0; i < 20; i++)
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    CHECK(ctl.state == DRIVE_FINISHED);
    CHECK_NEAR(cmd.throttle, 0.0, 1e-9);
    CHECK_NEAR(cmd.reverse, 0.0, 1e-9);
}

/*
** The halting-oscillation regression.
**
** Backing up to exactly the legal clearance and stopping there means the car
** coasts a few centimetres forward, drops below the line, backs up again, and
** never settles. Observed on a real track: eighty cycles of shuffling. The
** margin exists so that ordinary coasting after the reverse stops does not
** reset the count.
*/
static void test_halting_tolerates_coasting(void)
{
    config_t cfg;
    controller_t ctl;
    scan_t scan;
    gap_t gap;
    command_t cmd;
    int reversals = 0;

    plain_config(&cfg);
    controller_init(&ctl);
    ctl.state = DRIVE_HALTING;
    memset(&gap, 0, sizeof(gap));

    /* Back up until the margin is reached. */
    build_scan(&scan, &cfg, cfg.deadend_clearance_mm * 1.3);
    controller_step(&ctl, &scan, &gap, &cfg, &cmd);
    /* Then coast forward a little, staying legal but below the margin. */
    build_scan(&scan, &cfg, cfg.deadend_clearance_mm * 1.05);
    for (int i = 0; i < 15; i++) {
        controller_step(&ctl, &scan, &gap, &cfg, &cmd);
        if (cmd.reverse > 0.0)
            reversals++;
    }
    CHECK(ctl.state == DRIVE_FINISHED);
    /* And it got there without shuffling back and forth. */
    CHECK(reversals == 0);
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
    test_never_drives_without_a_gap();
    test_no_dead_zone_between_brake_and_reverse();
    test_stuck_without_a_gap_reverses();
    test_single_blind_cycle_is_ignored();
    test_repeated_failed_recoveries_end_the_race();
    test_clean_racing_clears_the_recovery_count();
    test_reversing_when_too_close();
    test_halting_backs_off_to_legal_clearance();
    test_halting_tolerates_coasting();
}
