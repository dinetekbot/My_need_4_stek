/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** control.c -- the driving laws and the state machine that supervises them.
**
** The steering law is a textbook PD controller. The proportional term turns
** the wheels in proportion to how far off-target the car is pointing; alone,
** it overshoots and the car weaves down the straight. The derivative term
** looks at how fast the error is shrinking and eases off early, which is what
** damps the weave. There is no integral term on purpose: the steady-state
** error we would be correcting is not constant, and an integrator with a
** rate-limited actuator winds up and takes over the car in the first hairpin.
*/

#include <math.h>
#include <string.h>

#include "control.h"
#include "util.h"

/* Leaving the reversing state at exactly the distance that triggered it makes
** the car oscillate between forward and reverse. We require noticeably more
** room than that before resuming. */
static const double REVERSE_EXIT_FACTOR = 1.8;
/* Cycles spent stationary before we accept that the car has actually stopped
** and the simulation can be ended. */
static const int HALT_SETTLE_CYCLES = 5;
/*
** While halting we back up to this multiple of the required clearance before
** starting to count settling cycles.
**
** Without the margin the car backs up to exactly the legal distance, stops
** reversing, coasts forward a few centimetres, drops below the line, backs up
** again -- and never settles. Observed on a real track: eighty cycles of
** shuffling between 950mm and 1150mm before the count happened to complete.
*/
static const double HALT_CLEARANCE_MARGIN = 1.25;
/* Clearance gain, in millimetres, that counts as reversing still working. */
static const double HALT_PROGRESS_MM = 20.0;
/* Consecutive cycles of getting nowhere before we settle for what we have. */
static const int HALT_STALL_CYCLES = 8;

void controller_init(controller_t *ctl)
{
    if (ctl == NULL)
        return;
    memset(ctl, 0, sizeof(*ctl));
    ctl->state = DRIVE_RACING;
}

const char *drive_state_name(drive_state_t state)
{
    switch (state) {
    case DRIVE_RACING:    return "RACING";
    case DRIVE_REVERSING: return "REVERSING";
    case DRIVE_HALTING:   return "HALTING";
    case DRIVE_FINISHED:  return "FINISHED";
    default:              return "UNKNOWN";
    }
}

double heading_error(const gap_t *gap)
{
    double half_fov = N4S_LIDAR_FOV_DEG / 2.0;

    if (gap == NULL || !gap->found)
        return 0.0;
    /* Normalising by the half field of view makes the gains independent of
    ** the sensor: swap in a 90 degree lidar and the tuning still holds. */
    return clampd(gap->target_angle / half_fov, -1.0, 1.0);
}

double centring_error(const scan_t *scan)
{
    double total;

    if (scan == NULL)
        return 0.0;
    total = scan->left_mm + scan->right_mm;
    if (total < 1e-6)
        return 0.0;
    /* Positive when the right side is more open, i.e. when the car is
    ** drifting towards the left wall and should ease right. */
    return clampd((scan->right_mm - scan->left_mm) / total, -1.0, 1.0);
}

double steering_command(controller_t *ctl, const scan_t *scan,
    const gap_t *gap, const config_t *cfg)
{
    double error;
    double derivative;
    double command;

    if (ctl == NULL || cfg == NULL)
        return 0.0;
    error = heading_error(gap);
    /* One cycle is the time unit, so the derivative is simply the change in
    ** error since last time. The loop runs at a steady rate because every
    ** cycle issues the same number of round trips. */
    derivative = error - ctl->last_error;
    command = cfg->steer_kp * error
        + cfg->steer_kd * derivative
        + cfg->steer_balance * centring_error(scan);
    command = clampd(command, -1.0, 1.0);
    command = rate_limit(command, ctl->last_wheels, cfg->steer_rate_limit);
    ctl->last_error = error;
    ctl->last_wheels = command;
    return command;
}

double throttle_command(const scan_t *scan, double wheels, const config_t *cfg)
{
    double power;

    if (scan == NULL || cfg == NULL)
        return 0.0;
    /* Hard stop first: no amount of clear space on the sides justifies
    ** driving into the wall in front. */
    if (scan->forward_mm < cfg->brake_dist_mm)
        return 0.0;
    power = map_range(scan->forward_mm, cfg->dist_slow_mm, cfg->dist_fast_mm,
        cfg->throttle_min, cfg->throttle_max);
    /* Steering hard and accelerating hard at the same time is how the car
    ** ends up sideways. Trade one for the other. */
    power *= 1.0 - cfg->turn_penalty * fabs(wheels);
    return clampd(power, 0.0, cfg->throttle_max);
}

/*
** Racing.
**
** The governing rule is that the car must never sit still doing nothing. An
** earlier version had two separate thresholds -- cut the throttle below
** `brake_dist`, start reversing below `reverse_dist` -- which left a band
** between them where the car was neither driving nor recovering. On a real
** track it parked itself in that band, 300mm from a wall, and waited there
** until a timer expired. Twenty wasted cycles, and nothing in the telemetry
** said why.
**
** So progress is measured, not assumed: any cycle that does not actually
** command forward motion counts as stuck, whatever the reason, and enough of
** them triggers a recovery.
*/
static void step_racing(controller_t *ctl, const scan_t *scan,
    const gap_t *gap, const config_t *cfg, command_t *out)
{
    bool moving;

    if (gap->found) {
        out->wheels = steering_command(ctl, scan, gap, cfg);
        out->throttle = throttle_command(scan, out->wheels, cfg);
        /* Closer than the braking distance, but there is still a way out.
        ** Crawl rather than stop: the car has to be rolling for the steering
        ** to do anything at all, and stopping dead in front of a wall we
        ** already know how to get around achieves nothing. */
        if (out->throttle <= 0.0 && scan->forward_mm >= cfg->reverse_dist_mm)
            out->throttle = cfg->crawl_power;
    } else {
        /*
        ** No gap, or too close to the wall for power to be safe. Hold the
        ** last steering angle and coast.
        **
        ** Note what does *not* happen here: the car does not creep forward.
        ** Driving into space the gap search could not find is driving blind,
        ** and it is how a recoverable situation becomes a collision.
        */
        out->wheels = ctl->last_wheels;
        out->throttle = 0.0;
    }
    moving = out->throttle > 0.0;
    if (moving) {
        ctl->stuck_cycles = 0;
        ctl->good_cycles++;
        /* A decent stretch of ordinary racing means whatever the car was
        ** caught in, it is out. Forgive the earlier recovery attempts. */
        if (ctl->good_cycles >= cfg->recovery_reset_cycles)
            ctl->recoveries = 0;
    } else {
        ctl->stuck_cycles++;
        ctl->good_cycles = 0;
    }
    if (scan->forward_mm >= cfg->reverse_dist_mm
        && ctl->stuck_cycles < cfg->blind_cycles)
        return;
    /* Recovery needed. Repeated recoveries without a clean stretch in
    ** between mean backing up is not working: this is a dead end. */
    if (ctl->recoveries >= cfg->deadend_attempts) {
        ctl->state = DRIVE_HALTING;
        ctl->halt_cycles = 0;
        return;
    }
    ctl->state = DRIVE_REVERSING;
    ctl->recoveries++;
    ctl->reverse_cycles = 0;
}

/*
** Reversing: steer *against* the gap. Backing up with the wheels turned left
** swings the nose right, so to end up pointing at an opening on the left the
** wheels go right while the car is going backwards.
*/
static void step_reversing(controller_t *ctl, const scan_t *scan,
    const gap_t *gap, const config_t *cfg, command_t *out)
{
    ctl->reverse_cycles++;
    /* Out of trouble: room ahead *and* somewhere to aim. Requiring both
    ** stops the car from lurching forward into the same wall again. */
    if (gap->found
        && scan->forward_mm > cfg->reverse_dist_mm * REVERSE_EXIT_FACTOR) {
        ctl->state = DRIVE_RACING;
        ctl->stuck_cycles = 0;
        return;
    }
    /* Backing up is not helping either. */
    if (ctl->reverse_cycles >= cfg->max_reverse_cycles) {
        ctl->state = DRIVE_HALTING;
        ctl->halt_cycles = 0;
        return;
    }
    out->wheels = -heading_error(gap);
    out->reverse = cfg->reverse_power;
    ctl->last_wheels = out->wheels;
}

/*
** Halting: come to rest, but not with the bumper against the wall -- the
** rules require at least three feet of clearance before the simulation is
** ended, so we creep backwards until we have it, with a margin.
*/
static void step_halting(controller_t *ctl, const scan_t *scan,
    const config_t *cfg, command_t *out)
{
    double target = cfg->deadend_clearance_mm * HALT_CLEARANCE_MARGIN;
    bool legal = scan->forward_mm >= cfg->deadend_clearance_mm;

    /* Track whether backing up is still buying clearance. */
    if (scan->forward_mm > ctl->halt_best_mm + HALT_PROGRESS_MM) {
        ctl->halt_best_mm = scan->forward_mm;
        ctl->halt_stalls = 0;
    } else {
        ctl->halt_stalls++;
    }
    /*
    ** Still backing up towards the margin -- unless reversing has stopped
    ** achieving anything.
    **
    ** There is no rear sensor. The car reverses blind, and in a corner it
    ** ends up wedged against a wall it cannot see, with the forward distance
    ** stuck short of the margin. Observed on a real track: 250 cycles of
    ** reversing against something at a constant 1125mm. So once reversing
    ** stops gaining ground, we settle for the clearance we have, provided it
    ** is legal.
    */
    if (ctl->halt_cycles == 0 && scan->forward_mm < target
        && !(legal && ctl->halt_stalls >= HALT_STALL_CYCLES)) {
        out->reverse = cfg->reverse_power;
        return;
    }
    /* Settling, but coasted back below the legal line: recover the margin
    ** and start the count again. */
    if (scan->forward_mm < cfg->deadend_clearance_mm) {
        out->reverse = cfg->reverse_power;
        ctl->halt_cycles = 0;
        return;
    }
    ctl->halt_cycles++;
    if (ctl->halt_cycles >= HALT_SETTLE_CYCLES)
        ctl->state = DRIVE_FINISHED;
}

void controller_step(controller_t *ctl, const scan_t *scan, const gap_t *gap,
    const config_t *cfg, command_t *out)
{
    if (ctl == NULL || scan == NULL || gap == NULL || cfg == NULL
        || out == NULL)
        return;
    memset(out, 0, sizeof(*out));
    ctl->cycles++;
    switch (ctl->state) {
    case DRIVE_RACING:
        step_racing(ctl, scan, gap, cfg, out);
        break;
    case DRIVE_REVERSING:
        step_reversing(ctl, scan, gap, cfg, out);
        break;
    case DRIVE_HALTING:
        step_halting(ctl, scan, cfg, out);
        break;
    case DRIVE_FINISHED:
    default:
        break;
    }
}
