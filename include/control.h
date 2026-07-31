/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** control.h -- deciding what to do with the free space we found.
**
** Two independent laws, then a state machine that supervises them:
**   steering  -- a PD controller on the heading error, plus a centring term
**   throttle  -- a distance-to-power curve, cut down in the corners
*/

#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>

#include "config.h"
#include "perception.h"

/* What the car is currently trying to do. */
typedef enum {
    DRIVE_RACING = 0,  /* normal operation: follow the gap                 */
    DRIVE_REVERSING,   /* too close to a wall, back up and re-aim          */
    DRIVE_HALTING,     /* dead end: brake, then end the simulation         */
    DRIVE_FINISHED     /* track cleared or lap limit reached               */
} drive_state_t;

/* One cycle's worth of actuator commands. */
typedef struct {
    double wheels;    /* WHEELS_DIR,    [-1;1], negative steers left */
    double throttle;  /* CAR_FORWARD,   [0;1]                       */
    double reverse;   /* CAR_BACKWARDS, [0;1]                       */
} command_t;

/* Everything the controller remembers between cycles. */
typedef struct {
    drive_state_t state;
    double        last_wheels;
    double        last_error;    /* previous heading error, for the D term */
    int           blind_cycles;  /* consecutive cycles without a gap       */
    int           halt_cycles;   /* consecutive cycles spent stationary    */
    long          cycles;        /* total cycles executed                  */
} controller_t;

void controller_init(controller_t *ctl);

/*
** Heading error in [-1;1]: 0 means the gap is dead ahead, -1 means it is at
** the far left edge of the cone. Normalising by the half-FOV keeps the gains
** meaningful whatever the sensor's field of view.
*/
double heading_error(const gap_t *gap);

/*
** Left/right imbalance in [-1;1]. Positive when there is more room on the
** right. Follow-the-gap alone will happily shave a wall it is not aimed at;
** this term nudges the car back towards the middle of the corridor.
*/
double centring_error(const scan_t *scan);

/*
** The steering law. Returns WHEELS_DIR, already clamped and rate limited.
** Updates ctl->last_error and ctl->last_wheels.
*/
double steering_command(controller_t *ctl, const scan_t *scan,
    const gap_t *gap, const config_t *cfg);

/*
** The throttle law. Full power on a clear straight, tapering to
** cfg->throttle_min as the wall ahead approaches, and cut further the harder
** we are steering. Returns CAR_FORWARD in [0;1].
*/
double throttle_command(const scan_t *scan, double wheels, const config_t *cfg);

/*
** Run one decision cycle: update the state machine and fill `out`.
**
** On the cycle where the state changes, `out` is left at zero -- the car
** coasts for one cycle rather than acting on a decision made by the state it
** is leaving. Race-ending events (track cleared, lap limit) are noticed by
** the caller, which owns the protocol; the controller only handles the
** situations it can see in the scan.
*/
void controller_step(controller_t *ctl, const scan_t *scan, const gap_t *gap,
    const config_t *cfg, command_t *out);

/* Human readable state name, for telemetry. */
const char *drive_state_name(drive_state_t state);

#endif /* CONTROL_H */
