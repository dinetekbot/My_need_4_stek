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
    double brake_dist_mm;     /* below this, cut the throttle entirely     */
    double reverse_dist_mm;   /* below this, back up out of trouble        */
    double reverse_power;     /* CAR_BACKWARDS power used when reversing   */

    /* ---- race management ----------------------------------------------- */

    /* A dead end is declared when no drivable gap is found for this many
    ** consecutive cycles. One bad scan is noise; ten in a row is a wall. */
    int    deadend_cycles;
    /* We must come to rest at least this far from the wall before ending
    ** the simulation in a dead end. The rulebook says three feet. */
    double deadend_clearance_mm;
    /* Stop after this many completed laps. 0 means "run until told to stop". */
    int    lap_limit;

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
    ** See docs/03-perception.md for the experiment that settles it.
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

/* Print the full option list on stderr. */
void config_usage(const char *program_name);

/* Dump the active configuration on stderr, one key per line. */
void config_dump(const config_t *cfg);

#endif /* CONFIG_H */
