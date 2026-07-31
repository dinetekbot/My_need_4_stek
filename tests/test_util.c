/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_util.c -- the numeric helpers.
*/

#include <stdio.h>

#include "test.h"
#include "util.h"

static void test_clamp(void)
{
    CHECK_NEAR(clampd(0.5, 0.0, 1.0), 0.5, 1e-9);
    CHECK_NEAR(clampd(-3.0, 0.0, 1.0), 0.0, 1e-9);
    CHECK_NEAR(clampd(9.0, 0.0, 1.0), 1.0, 1e-9);
}

/*
** The bug this function exists to prevent.
**
** An earlier version of this project mapped a lidar distance onto a throttle
** value with an unclamped linear formula: (x - a) / (b - a) * (d - c) + c.
** With a=10, b=50, c=0, d=0.1 and a real distance of 500mm that returns
** 1.225 -- a throttle outside [0;1], which the API rejects outright. The car
** did not accelerate, and nothing in the code said why.
*/
static void test_map_range_is_clamped(void)
{
    CHECK_NEAR(map_range(500.0, 10.0, 50.0, 0.0, 0.1), 0.1, 1e-9);
    CHECK_NEAR(map_range(-100.0, 10.0, 50.0, 0.0, 0.1), 0.0, 1e-9);
    CHECK_NEAR(map_range(30.0, 10.0, 50.0, 0.0, 1.0), 0.5, 1e-9);
    /* A degenerate input range must not divide by zero. */
    CHECK_NEAR(map_range(5.0, 10.0, 10.0, 0.2, 0.9), 0.2, 1e-9);
    /* Descending output ranges are legal. */
    CHECK_NEAR(map_range(30.0, 10.0, 50.0, 1.0, 0.0), 0.5, 1e-9);
}

static void test_rate_limit(void)
{
    CHECK_NEAR(rate_limit(1.0, 0.0, 0.25), 0.25, 1e-9);
    CHECK_NEAR(rate_limit(-1.0, 0.0, 0.25), -0.25, 1e-9);
    CHECK_NEAR(rate_limit(0.1, 0.0, 0.25), 0.1, 1e-9);
    /* A non-positive limit disables the feature rather than freezing the
    ** actuator, which would be a very confusing way to fail. */
    CHECK_NEAR(rate_limit(1.0, 0.0, 0.0), 1.0, 1e-9);
}

static void test_parse_double(void)
{
    double value = 0.0;

    CHECK(parse_double("50.3", &value));
    CHECK_NEAR(value, 50.3, 1e-9);
    CHECK(parse_double("-25.1", &value));
    CHECK_NEAR(value, -25.1, 1e-9);
    CHECK(parse_double("  7 ", &value));
    /* These must be rejected: the parser uses this to tell a numeric protocol
    ** field apart from free text, so "12abc" passing would swallow part of
    ** ADDITIONAL_INFO into the payload. */
    CHECK(!parse_double("No further info", NULL));
    CHECK(!parse_double("12abc", NULL));
    CHECK(!parse_double("", NULL));
    CHECK(!parse_double(NULL, NULL));
    CHECK(!parse_double("[115s,559291238ns]", NULL));
}

static void test_aggregates(void)
{
    double values[5] = {4.0, 1.0, 9.0, 3.0, 2.0};

    CHECK_NEAR(mean_range(values, 0, 5), 3.8, 1e-9);
    CHECK(argmin_range(values, 0, 5) == 1);
    CHECK(argmax_range(values, 0, 5) == 2);
    /* Sub-ranges, as used for the left and right thirds of a scan. */
    CHECK(argmin_range(values, 2, 5) == 4);
    /* An empty range must not read out of bounds. */
    CHECK_NEAR(mean_range(values, 2, 2), 0.0, 1e-9);
    CHECK(argmin_range(NULL, 0, 5) == 0);
}

void test_util(void)
{
    printf("util\n");
    test_clamp();
    test_map_range_is_clamped();
    test_rate_limit();
    test_parse_double();
    test_aggregates();
}
