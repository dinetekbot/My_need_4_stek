/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_perception.c -- scan geometry, cleaning, safety bubble, gap finding.
**
** These are the tests that make the driving code debuggable. A car that
** understeers in a simulator tells you nothing about which of five stages is
** wrong; a failing assertion on a hand-written scan tells you exactly.
*/

#include <math.h>
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
    /* Nothing at all in the obstacle's own direction. */
    CHECK_NEAR(scan.ranges[5], 0.0, 1e-9);
    /* Its neighbours are strongly attenuated but not erased: attenuating
    ** keeps the distance field continuous, so the forbidden sector cannot
    ** teleport when the closest ray flips between two near-equidistant
    ** obstacles. */
    CHECK(scan.ranges[4] > 0.0 && scan.ranges[4] < 0.1 * 3000.0);
    CHECK(scan.ranges[6] > 0.0 && scan.ranges[6] < 0.1 * 3000.0);
    /* And the attenuation eases off monotonically with distance from it. */
    CHECK(scan.ranges[6] < scan.ranges[7]);
    CHECK(scan.ranges[7] < scan.ranges[8]);
    /* The cap guarantees the far side of the cone survives untouched. */
    CHECK_NEAR(scan.ranges[N4S_LIDAR_RAYS - 1], 3000.0, 1e-9);
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
    ** swing the target across the cone. The aim is a continuous function of
    ** the distances now, so it may move -- but by far less than the sensor
    ** can resolve. */
    CHECK_NEAR(first.target_angle, second.target_angle,
        N4S_LIDAR_FOV_DEG / N4S_LIDAR_RAYS / 10.0);
}

/*
** The aim is an angle, not a ray.
**
** The old version returned a ray index, so the answer could only be a
** multiple of 1.9 degrees and stepped whenever the chosen ray changed.
** Measured on a real track, that meant 8 to 10 degree jumps per cycle and a
** steering reversal on half of them.
*/
static void test_target_is_continuous(void)
{
    config_t cfg;
    scan_t scan;
    gap_t gap;
    double raw[N4S_LIDAR_RAYS];
    double previous = 0.0;
    int moved = 0;
    int between_rays = 0;

    plain_config(&cfg);
    /* Deepen one edge of the opening gradually, within the band that counts,
    ** and follow the aim. An index-based answer sits still and then jumps; a
    ** centroid drifts. */
    for (int step = 0; step < 20; step++) {
        bool snapped = false;

        fill(raw, 2800.0);
        for (size_t i = 0; i < 14; i++)
            raw[i] = 300.0;
        raw[14] = 2405.0 + step * 20.0;
        scan_build(&scan, raw, &cfg);
        scan_find_gap(&scan, &cfg, &gap);
        CHECK(gap.found);
        if (step > 0) {
            /* Never a jump: each increment moves the aim well under one ray,
            ** where an index-based answer moved by zero or a whole ray. */
            CHECK(fabs(gap.target_angle - previous)
                < N4S_LIDAR_FOV_DEG / N4S_LIDAR_RAYS);
            if (fabs(gap.target_angle - previous) > 1e-9)
                moved++;
        }
        for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
            if (fabs(gap.target_angle - scan.angles[i]) < 1e-9)
                snapped = true;
        if (!snapped)
            between_rays++;
        previous = gap.target_angle;
    }
    /* It moved on most steps, and spent most of them pointing between rays --
    ** neither is possible when the answer is a ray index. */
    CHECK(moved >= 15);
    CHECK(between_rays >= 15);
}

/*
** Do not switch openings for a marginal gain: two near-equal gaps otherwise
** flip from cycle to cycle and the target teleports across the cone.
*/
static void test_gap_choice_has_hysteresis(void)
{
    config_t cfg;
    scan_t scan;
    gap_t gap;
    double raw[N4S_LIDAR_RAYS];
    double committed;

    plain_config(&cfg);
    fill(raw, 400.0);
    for (size_t i = 2; i < 9; i++)
        raw[i] = 2600.0;
    for (size_t i = 22; i < 29; i++)
        raw[i] = 2500.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap(&scan, &cfg, &gap);
    CHECK(gap.found);
    CHECK(gap.target_angle < 0.0);        /* the deeper one, on the left */
    committed = gap.target_angle;

    /* The right one becomes marginally better. Without hysteresis the target
    ** would jump the width of the cone for almost nothing. */
    for (size_t i = 22; i < 29; i++)
        raw[i] = 2700.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap_from(&scan, &cfg, committed, &gap);
    CHECK(gap.target_angle < 0.0);

    /* A decisively better opening still wins. */
    for (size_t i = 22; i < 29; i++)
        raw[i] = 3010.0;
    for (size_t i = 2; i < 9; i++)
        raw[i] = 1000.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap_from(&scan, &cfg, committed, &gap);
    CHECK(gap.target_angle > 0.0);

    /* And with hysteresis off, the marginal gain is enough to switch. */
    cfg.gap_hysteresis = 0.0;
    fill(raw, 400.0);
    for (size_t i = 2; i < 9; i++)
        raw[i] = 2600.0;
    for (size_t i = 22; i < 29; i++)
        raw[i] = 2700.0;
    scan_build(&scan, raw, &cfg);
    scan_find_gap_from(&scan, &cfg, committed, &gap);
    CHECK(gap.target_angle > 0.0);
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
    test_target_is_continuous();
    test_gap_choice_has_hysteresis();
}
