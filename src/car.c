/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** car.c -- the driving law.
**
** The idea is intentionally small. Look at how far the road reaches straight
** ahead, pick a power and a steering lock from that distance, and turn the
** lock towards the more open side of the corridor. No map, no state, no
** controller tuning: just a dozen thresholds that happen to work.
**
** I have kept the thresholds in tables rather than chains of if/else. There
** are two of them, they are read in the same order every cycle, and when a
** value needs changing there is exactly one line to change.
*/

#include <stddef.h>

#include "car.h"

typedef struct {
    double distance_mm;  /* smallest clearance this tier covers */
    double value;        /* power or steering lock to use       */
} tier_t;

/* Clear road ahead means full power; a wall means barely enough to creep. */
static const tier_t THROTTLE_TIERS[] = {
    {2000.0, 1.0},
    {1500.0, 0.8},
    {1000.0, 0.5},
    { 600.0, 0.4},
    { 400.0, 0.2},
    {   0.0, 0.1}
};

/*
** The closer the wall, the more lock we are willing to apply. Far away the
** car only needs a feather of correction; near an obstacle it needs a real
** turn, so the lock grows as the clearance shrinks.
*/
static const tier_t STEERING_TIERS[] = {
    {1500.0, 0.005},
    {1000.0, 0.05},
    { 600.0, 0.1},
    { 400.0, 0.2},
    { 200.0, 0.3},
    {   0.0, 0.5}
};

static double lookup(const tier_t *tiers, size_t count, double distance)
{
    for (size_t i = 0; i < count; i++)
        if (distance >= tiers[i].distance_mm)
            return tiers[i].value;
    /* The last tier starts at 0, so this is only reachable for negative
    ** readings. Treat those as a wall rather than as open road. */
    return tiers[count - 1].value;
}

double car_throttle(double forward_mm)
{
    return lookup(THROTTLE_TIERS,
        sizeof(THROTTLE_TIERS) / sizeof(THROTTLE_TIERS[0]), forward_mm);
}

double car_steering(double forward_mm, double left_mm, double right_mm)
{
    double lock = lookup(STEERING_TIERS,
        sizeof(STEERING_TIERS) / sizeof(STEERING_TIERS[0]), forward_mm);

    /* More room on the left means turn left, and more room on the right means
    ** turn right. Equal distances keep the old positive direction, which is
    ** what the simulator expects. */
    if (left_mm < right_mm)
        return -lock;
    return lock;
}

void car_step(const double ranges[N4S_LIDAR_RAYS], car_command_t *out)
{
    double forward;
    double left;
    double right;

    if (ranges == NULL || out == NULL)
        return;
    forward = ranges[CAR_FORWARD_RAY];
    left = ranges[CAR_LEFT_RAY];
    right = ranges[CAR_RIGHT_RAY];
    out->throttle = car_throttle(forward);
    out->wheels = car_steering(forward, left, right);
}
