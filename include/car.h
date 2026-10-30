/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** car.h -- the driving law.
**
** The whole decision fits in two tables: one maps the distance straight
** ahead to engine power, the other maps it to how much lock to apply. The
** direction of that lock comes from the two edge rays, which is enough to
** keep the car in the middle of the corridor.
*/

#ifndef CAR_H
#define CAR_H

#include "n4s.h"

/* One cycle's actuator values. */
typedef struct {
    double wheels;    /* WHEELS_DIR,   [-1;1] */
    double throttle;  /* CAR_FORWARD,  [0;1]  */
} car_command_t;

/*
** The raw rays this algorithm reads.
**
** Ray 15 sits just left of dead centre, ray 1 is near the left wall and ray
** 31 is near the right wall. The numbers are the ones the original N4S
** implementation used, and they work because the sensor order is stable.
*/
#define CAR_FORWARD_RAY 15
#define CAR_LEFT_RAY     1
#define CAR_RIGHT_RAY   31

/* Power for a given forward clearance, from the tier table. */
double car_throttle(double forward_mm);

/* Steering for a given clearance and pair of edge distances. */
double car_steering(double forward_mm, double left_mm, double right_mm);

/* Fill `out` from one raw scan. */
void car_step(const double ranges[N4S_LIDAR_RAYS], car_command_t *out);

#endif /* CAR_H */
