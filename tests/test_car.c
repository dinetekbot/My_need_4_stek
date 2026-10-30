/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_car.c -- assert the driving law at every tier boundary.
**
** The algorithm is only a pair of tables, so the interesting cases are the
** edges: one millimetre below a threshold must select a different tier than
** the threshold itself. A test that only checks the middle of a tier would
** pass even if every boundary were shifted.
*/

#include "car.h"
#include "test.h"

static void test_throttle_tiers(void)
{
    CHECK_NEAR(car_throttle(3000.0), 1.0, 1e-9);
    CHECK_NEAR(car_throttle(2000.0), 1.0, 1e-9);
    CHECK_NEAR(car_throttle(1999.0), 0.8, 1e-9);
    CHECK_NEAR(car_throttle(1500.0), 0.8, 1e-9);
    CHECK_NEAR(car_throttle(1499.0), 0.5, 1e-9);
    CHECK_NEAR(car_throttle(1000.0), 0.5, 1e-9);
    CHECK_NEAR(car_throttle( 999.0), 0.4, 1e-9);
    CHECK_NEAR(car_throttle( 600.0), 0.4, 1e-9);
    CHECK_NEAR(car_throttle( 599.0), 0.2, 1e-9);
    CHECK_NEAR(car_throttle( 400.0), 0.2, 1e-9);
    CHECK_NEAR(car_throttle( 399.0), 0.1, 1e-9);
    CHECK_NEAR(car_throttle(   0.0), 0.1, 1e-9);
    CHECK_NEAR(car_throttle(-100.0), 0.1, 1e-9);
}

static void test_steering_magnitude(void)
{
    CHECK_NEAR(car_steering(3000.0, 1000.0, 500.0),  0.005, 1e-9);
    CHECK_NEAR(car_steering(1500.0, 1000.0, 500.0),  0.005, 1e-9);
    CHECK_NEAR(car_steering(1499.0, 1000.0, 500.0),  0.05,  1e-9);
    CHECK_NEAR(car_steering(1000.0, 1000.0, 500.0),  0.05,  1e-9);
    CHECK_NEAR(car_steering( 999.0, 1000.0, 500.0),  0.1,   1e-9);
    CHECK_NEAR(car_steering( 600.0, 1000.0, 500.0),  0.1,   1e-9);
    CHECK_NEAR(car_steering( 599.0, 1000.0, 500.0),  0.2,   1e-9);
    CHECK_NEAR(car_steering( 400.0, 1000.0, 500.0),  0.2,   1e-9);
    CHECK_NEAR(car_steering( 399.0, 1000.0, 500.0),  0.3,   1e-9);
    CHECK_NEAR(car_steering( 200.0, 1000.0, 500.0),  0.3,   1e-9);
    CHECK_NEAR(car_steering( 199.0, 1000.0, 500.0),  0.5,   1e-9);
    CHECK_NEAR(car_steering(   0.0, 1000.0, 500.0),  0.5,   1e-9);
}

static void test_steering_direction(void)
{
    /* More room on the left selects the positive lock. */
    CHECK_NEAR(car_steering(2000.0, 2000.0, 500.0),  0.005, 1e-9);
    /* More room on the right selects the negative lock. */
    CHECK_NEAR(car_steering(2000.0, 500.0, 2000.0), -0.005, 1e-9);
    /* Symmetric corridor keeps the positive convention. */
    CHECK_NEAR(car_steering(2000.0, 1000.0, 1000.0), 0.005, 1e-9);
}

static void test_step_reads_the_agreed_rays(void)
{
    double ranges[N4S_LIDAR_RAYS];
    car_command_t cmd;

    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        ranges[i] = 42.0;
    ranges[CAR_FORWARD_RAY] = 2000.0;
    ranges[CAR_LEFT_RAY] = 1000.0;
    ranges[CAR_RIGHT_RAY] = 500.0;
    car_step(ranges, &cmd);
    CHECK_NEAR(cmd.throttle, 1.0, 1e-9);
    CHECK_NEAR(cmd.wheels, 0.005, 1e-9);

    /* A value in another ray must not influence the decision. */
    ranges[CAR_FORWARD_RAY] = 300.0;
    ranges[CAR_LEFT_RAY] = 100.0;
    ranges[CAR_RIGHT_RAY] = 900.0;
    ranges[0] = 5000.0;
    car_step(ranges, &cmd);
    CHECK_NEAR(cmd.throttle, 0.1, 1e-9);
    CHECK_NEAR(cmd.wheels, -0.3, 1e-9);
}

void test_car(void)
{
    printf("car\n");
    test_throttle_tiers();
    test_steering_magnitude();
    test_steering_direction();
    test_step_reads_the_agreed_rays();
}
