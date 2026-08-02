/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** config.h -- every tunable number in one place.
**
** Nothing in the driving code hard-codes a threshold. Each value below is a
** knob you can turn from the command line, which is what makes the car
** tunable per track instead of rewritten per track. See docs/06-tuning.md.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>

typedef struct {
    /* ---- perception ---------------------------------------------------- */

    /* Rays closer than this are treated as "wall right here". */
    double lidar_floor_mm;
    /* Width of the moving average applied to the raw scan, in rays.
    ** 1 disables smoothing. Odd values keep the window centred. */
    int    smooth_window;
    /* Radius, in millimetres, of the exclusion bubble drawn around the
    ** closest obstacle. Roughly "half the car width plus a safety margin".
    ** The car's real dimensions are deliberately unknown, so this is
    ** calibrated by observation rather than derived. */
    double bubble_mm;
    /* A ray must reach at least this far to count as drivable space. */
    double gap_threshold_mm;
    /*
    ** How much better a rival opening must score before the car switches to
    ** it, as a fraction of the incumbent's score. 0 disables the preference.
    **
    ** Without it, the choice of opening flips between two near-equal
    ** candidates as the car moves and the heading target teleports across the
    ** cone. Measured before it existed: the target moved 8 to 10 degrees per
    ** cycle on average against a sensor that resolves 1.9, and the steering
    ** reversed sign on half of all cycles.
    */
    double gap_hysteresis;

    /* ---- steering ------------------------------------------------------ */

    double steer_kp;          /* proportional gain on the heading error   */
    double steer_kd;          /* derivative gain, damps the oscillation   */
    double steer_balance;     /* weight of the left/right centring term   */
    double steer_rate_limit;  /* max change of WHEELS_DIR per cycle       */

    /* ---- throttle ------------------------------------------------------ */

    double throttle_min;      /* never crawl slower than this while racing */
    double throttle_max;      /* full power on a clear straight            */
    double dist_slow_mm;      /* forward distance mapping to throttle_min  */
    double dist_fast_mm;      /* forward distance mapping to throttle_max  */
    double turn_penalty;      /* how much a full lock cuts the throttle    */
    double brake_dist_mm;     /* below this, drop to a crawl               */
    /*
    ** Power used when closer than brake_dist but still able to see a way out.
    **
    ** Not zero, and that matters. A stationary car with its wheels turned
    ** goes nowhere: cutting the throttle completely near a wall prevents the
    ** exact manoeuvre needed to get round it, and the car sits in the corner
    ** steering at an opening it never reaches. Just enough power to rotate,
    ** not enough to hurt if the gap turns out to be wrong.
    */
    double crawl_power;
    double reverse_dist_mm;   /* below this, back up out of trouble        */
    double reverse_power;     /* CAR_BACKWARDS power used when reversing   */

    /* ---- race management ----------------------------------------------- */

    /*
    ** Consecutive gapless cycles before the car stops trying to go forward
    ** and backs up to look again. One bad scan is noise; a few in a row means
    ** the nose is pointed at something.
    **
    ** This must be small. Creeping forward while the gap search is returning
    ** nothing is driving blind, and the wall does not move out of the way
    ** while you wait to be sure.
    */
    int    blind_cycles;
    /*
    ** The escape manoeuvre: a three-point turn.
    **
    ** Reversing in a straight line is useless here, and that is not an
    ** opinion -- it is geometry. Backing straight up translates the car
    ** without rotating it, so when it drives forward again it faces exactly
    ** the same wall from slightly further away. Measured over the real runs,
    ** 128 of 294 reversing cycles applied zero steering and achieved
    ** precisely that.
    **
    ** A car with Ackermann steering cannot turn on the spot, so the only way
    ** to change its heading is to alternate: back up with the wheels one way,
    ** pull forward with them the other. Each pair rotates it.
    */
    int    escape_reverse_cycles;  /* length of the backing-up phase   */
    int    escape_forward_cycles;  /* length of the pulling-forward phase */
    int    escape_max_pairs;       /* pairs before admitting a dead end */
    /* Number of reverse-and-retry attempts, without a clean stretch of
    ** racing in between, before declaring a dead end. */
    int    deadend_attempts;
    /* Untroubled racing cycles that clear the attempt counter: proof that
    ** the car got out of whatever it was stuck in rather than merely
    ** twitching between two flavours of stuck. */
    int    recovery_reset_cycles;
    /* We must come to rest at least this far from the wall before ending
    ** the simulation in a dead end. The rulebook says three feet. */
    double deadend_clearance_mm;
    /* Stop after this many completed laps. 0 means "run until told to stop". */
    int    lap_limit;

    /*
    ** Simulated seconds per control cycle that the distances above assume.
    **
    ** The braking distances are only meaningful relative to how far the car
    ** travels between two decisions, and that is not a property of the car --
    ** it is a property of how fast the simulator advances while we talk to
    ** it. Measured on the same machine and the same scene: 0.36 simulated
    ** seconds per cycle headless, 0.87 with the window open. The windowed
    ** car covers nearly two and a half times the ground between decisions,
    ** cannot brake for the first corner, and wedges itself -- which reads as
    ** "the robot does not move" because the lidar then stops changing.
    **
    ** So the pace is measured at startup and the distances are scaled by it.
    ** Set to 0 to disable the measurement and use the distances as written.
    */
    double pace_reference_s;

    /* ---- plumbing ------------------------------------------------------ */

    /*
    ** Left/right convention correction.
    **
    ** Two things are undocumented: whether lidar ray 0 points left or right,
    ** and whether a positive WHEELS_DIR steers right or left. That looks like
    ** four cases, but it is really two: mirroring the perception and
    ** mirroring the action cancel each other out. Only the combination is
    ** observable, so a single flag covers it -- if the car consistently
    ** steers into the wall it should be avoiding, set this and it is fixed,
    ** whichever of the two conventions was actually backwards.
    **
    ** It is implemented by reversing the scan, so every other function in the
    ** project can assume "index 0 is on the left, positive steers right".
    **
    ** Default: on. Measured against the N4S simulator, where leaving it off
    ** makes the car steer into the first corner it meets. Set --mirror=0 if
    ** your setup is wired the other way round. See docs/03-perception.md for
    ** the experiment.
    */
    bool   mirror;
    /* Print a telemetry line per cycle on stderr. */
    bool   verbose;
} config_t;

/* Populate `cfg` with the defaults documented above. */
void config_defaults(config_t *cfg);

/*
** Apply command line overrides. Returns 0 on success, -1 on a bad argument
** (a message is printed on stderr), 1 when --help was requested.
*/
int config_parse_args(config_t *cfg, int argc, char **argv);

/*
** Apply overrides from an environment variable holding a whitespace-separated
** option list, e.g. AI_OPTS="--verbose --gap-threshold=600".
**
** This exists because the launcher shipped with the simulator API runs the
** binary with no arguments and no way to add any. Without an environment
** channel, using the documented launcher means accepting the defaults and
** having no telemetry -- so tuning would require abandoning the official
** workflow, which is the wrong trade.
**
** Same return values as config_parse_args. Command line options are applied
** afterwards and therefore win.
*/
int config_parse_env(config_t *cfg, const char *variable);

/* Print the full option list on stderr. */
void config_usage(const char *program_name);

/* Dump the active configuration on stderr, one key per line. */
void config_dump(const config_t *cfg);

#endif /* CONFIG_H */
