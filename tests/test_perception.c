/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_perception.c -- scan geometry, cleaning, safety bubble, gap finding.
**
** These are the tests that make the driving code debuggable. A car that
** understeers in a simulator tells you nothing about which of five stages is
** wrong; a failing assertion on a hand-written scan tells you exactly.
*/

#include <stdio.h>
#include <string.h>

#include "perception.h"
#include "test.h"

/* Smoothing is disabled in most tests so that an assertion refers to the
** number that was put in, not to a filtered version of it. */
static void plain_config(config_t *cfg)
{
    config_defaults(cfg);
    cfg->smooth_window = 1;
    /* Pin the ray order rather than inheriting the platform default: these
    ** tests are about geometry, not about which way round the real sensor
    ** happens to be wired. */
    cfg->mirror = false;
}

static void fill(double *ranges, double value)
{
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        ranges[i] = value;
}

static void test_ray_angles(void)
{
    double step = N4S_LIDAR_FOV_DEG / (double)N4S_LIDAR_RAYS;

    /* Rays sample slice centres, so the outermost ray sits half a step inside
    ** the cone. Getting this wrong biases every steering command by half a
    ** ray's worth of angle. */
    CHECK_NEAR(ray_angle(0, false), -30.0 + step / 2.0, 1e-9);
    CHECK_NEAR(ray_angle(N4S_LIDAR_RAYS - 1, false), 30.0 - step / 2.0, 1e-9);
    /* The cone is symmetric. */
    CHECK_NEAR(ray_angle(0, false) + ray_angle(N4S_LIDAR_RAYS - 1, false),
        0.0, 1e-9);
    /* Mirroring is exactly a sign flip. */
    CHECK_NEAR(ray_angle(3, true), -ray_angle(3, false), 1e-9);
}

/*
** A faulty reading must never look like open road. The simulator reports
** zeros before the scene is running, and an optimistic substitution there
** means the car launches itself at a wall it cannot see.
*/
static void test_sanitising(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 1500.0);
    raw[6] = 99999.0;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.ranges[6], N4S_LIDAR_MAX_MM, 1e-9);
    CHECK_NEAR(scan.ranges[0], 1500.0, 1e-9);

    /* A run of dead rays has nothing to interpolate from, so it collapses to
    ** the floor: the pessimistic answer, which makes the car brake. */
    fill(raw, 1500.0);
    raw[10] = -12.0;
    raw[11] = 0.0;
    raw[12] = 0.0;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.ranges[11], cfg.lidar_floor_mm, 1e-9);
}

/*
** An isolated dropout is not a wall.
**
** Substituting "obstacle one millimetre away" for a single dead ray whose
** neighbours are healthy inflates the safety bubble to its maximum for that
** cycle and can erase an opening the car could have used. Measured on a real
** track: 9 of 254 cycles. Neighbouring rays are the better estimate.
*/
static void test_isolated_dropout_is_interpolated(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 1500.0);
    raw[8] = 1000.0;
    raw[9] = 0.0;      /* dropout between two healthy rays */
    raw[10] = 1400.0;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.ranges[9], 1200.0, 1e-9);
    /* And it no longer looks like the closest obstacle in the scan. */
    CHECK(scan.closest_mm > cfg.lidar_floor_mm);
}

static void test_mirroring(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 1500.0);
    raw[0] = 400.0;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.ranges[0], 400.0, 1e-9);

    cfg.mirror = true;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.ranges[N4S_LIDAR_RAYS - 1], 400.0, 1e-9);
    CHECK_NEAR(scan.ranges[0], 1500.0, 1e-9);
}

/*
** forward_mm is the minimum over the central rays, not their mean. A thin
** obstacle dead ahead averages away to a comfortable-looking distance, and
** the car drives into it under full power.
*/
static void test_forward_uses_minimum(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 3000.0);
    raw[16] = 400.0;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.forward_mm, 400.0, 1e-9);
    CHECK_NEAR(scan.closest_mm, 400.0, 1e-9);
    CHECK(scan.closest_index == 16);
}

static void test_side_summaries(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        raw[i] = (i < N4S_LIDAR_RAYS / 2) ? 800.0 : 2400.0;
    scan_build(&scan, raw, &cfg);
    CHECK_NEAR(scan.left_mm, 800.0, 1e-9);
    CHECK_NEAR(scan.right_mm, 2400.0, 1e-9);
}

static void test_smoothing(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    cfg.smooth_window = 3;
    fill(raw, 1000.0);
    raw[10] = 1300.0;
    scan_build(&scan, raw, &cfg);
    /* The spike is spread over its neighbours but the total is preserved. */
    CHECK_NEAR(scan.ranges[10], 1100.0, 1e-9);
    CHECK_NEAR(scan.ranges[9], 1100.0, 1e-9);
    CHECK_NEAR(scan.ranges[0], 1000.0, 1e-9);
}

static void test_bubble_erases_around_closest(void)
{
    config_t cfg;
    scan_t scan;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 3000.0);
    raw[5] = 300.0;
    scan_build(&scan, raw, &cfg);
    scan_apply_bubble(&scan, &cfg);
    CHECK_NEAR(scan.ranges[5], 0.0, 1e-9);
    CHECK_NEAR(scan.ranges[4], 0.0, 1e-9);
    CHECK_NEAR(scan.ranges[6], 0.0, 1e-9);
    /* The cap guarantees the far side of the cone survives, so a gap can
    ** still be found when a wall is right against the car. */
    CHECK(scan.ranges[N4S_LIDAR_RAYS - 1] > 0.0);
}

static void test_gap_on_the_open_side(void)
{
    config_t cfg;
    scan_t scan;
    gap_t gap;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    /* Wall on the right half, open road on the left. */
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        raw[i] = (i < 12) ? 2800.0 : 500.0;
    scan_build(&scan, raw, &cfg);
    scan_apply_bubble(&scan, &cfg);
    scan_find_gap(&scan, &cfg, &gap);
    CHECK(gap.found);
    CHECK(gap.target < 12);
    CHECK(gap.target_angle < 0.0);
    CHECK_NEAR(gap.depth_mm, 2800.0, 1e-9);
}

static void test_gap_centred_when_symmetric(void)
{
    config_t cfg;
    scan_t scan;
    gap_t gap;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    /* A straight corridor: walls on both edges, clear down the middle. */
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        raw[i] = (i < 6 || i >= N4S_LIDAR_RAYS - 6) ? 500.0 : 2800.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap(&scan, &cfg, &gap);
    CHECK(gap.found);
    /* Aiming down the middle of a symmetric corridor, within one ray. */
    CHECK_NEAR(gap.target_angle, 0.0, 2.0);
}

static void test_no_gap_when_boxed_in(void)
{
    config_t cfg;
    scan_t scan;
    gap_t gap;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 400.0);
    scan_build(&scan, raw, &cfg);
    scan_apply_bubble(&scan, &cfg);
    scan_find_gap(&scan, &cfg, &gap);
    CHECK(!gap.found);
    CHECK(gap.target_angle == 0.0);
}

/* The target must not flicker between two nearly equal rays: that is what
** makes a car wag its way down a straight. */
static void test_target_is_stable_on_a_plateau(void)
{
    config_t cfg;
    scan_t scan;
    gap_t first;
    gap_t second;
    double raw[N4S_LIDAR_RAYS];

    plain_config(&cfg);
    fill(raw, 2900.0);
    raw[8] = 2905.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap(&scan, &cfg, &first);
    raw[8] = 2900.0;
    raw[24] = 2905.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap(&scan, &cfg, &second);
    /* A 5mm difference at the far end of a uniform wall of readings must not
    ** swing the target from one side of the cone to the other. */
    CHECK_NEAR(first.target_angle, second.target_angle, 1e-9);
}

void test_perception(void)
{
    printf("perception\n");
    test_ray_angles();
    test_sanitising();
    test_isolated_dropout_is_interpolated();
    test_mirroring();
    test_forward_uses_minimum();
    test_side_summaries();
    test_smoothing();
    test_bubble_erases_around_closest();
    test_gap_on_the_open_side();
    test_gap_centred_when_symmetric();
    test_no_gap_when_boxed_in();
    test_target_is_stable_on_a_plateau();
}
