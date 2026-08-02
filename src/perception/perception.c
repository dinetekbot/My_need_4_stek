/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** perception.c -- from 32 numbers to "drive that way".
**
** The pipeline, in order:
**
**   raw scan -> normalise -> sanitise -> smooth -> summarise   (scan_build)
**            -> erase the danger zone                     (scan_apply_bubble)
**            -> pick the widest opening                     (scan_find_gap)
**
** Each stage is separately testable, which is the whole reason they are
** separate functions: you cannot debug a driving bug in a simulator by
** staring at it, but you can feed a hand-written scan to scan_find_gap and
** assert on the answer.
*/

#include <math.h>
#include <string.h>

#include "perception.h"
#include "util.h"

/* Rays considered "straight ahead" when judging braking distance. Five rays
** span about 9 degrees, narrow enough to ignore the walls of a corridor and
** wide enough not to miss a post. */
#define FORWARD_FIRST 14
#define FORWARD_LAST  19

/* How much shallower than the deepest ray a ray may be and still count as
** part of the target plateau. Aiming at the single deepest ray makes the
** target jump between two nearly equal rays from one cycle to the next, and
** the car wags its way down the straight. */
static const double PLATEAU_TOLERANCE_MM = 400.0;

double ray_angle(size_t index, bool inverted)
{
    double step = N4S_LIDAR_FOV_DEG / (double)N4S_LIDAR_RAYS;
    /* Rays sample the centre of each slice, so the first one is half a step
    ** inside the cone rather than exactly on its edge. */
    double angle = -(N4S_LIDAR_FOV_DEG / 2.0) + ((double)index + 0.5) * step;

    return inverted ? -angle : angle;
}

/*
** Copy the raw scan into the working buffer, reversing it when the sensor
** reports right-to-left. After this, index 0 is always the leftmost ray, and
** no other function in the project needs to know about the hardware order.
*/
static void normalise_order(double *dst, const double *raw, bool inverted)
{
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        dst[i] = inverted ? raw[N4S_LIDAR_RAYS - 1 - i] : raw[i];
}

static bool ray_is_valid(double value)
{
    return isfinite(value) && value > 0.0;
}

/*
** Force every reading into a usable range.
**
** A non-positive or non-finite reading is a sensor fault, not open space, and
** the two possible substitutions are not symmetric. Treating it as open road
** makes the car accelerate at something it cannot see. So the fallback is the
** floor distance -- "obstacle right here" -- and the failure mode is braking.
**
** But an *isolated* dead ray, with healthy neighbours either side, is a
** dropout rather than a wall, and pretending it is an obstacle one millimetre
** away is its own kind of wrong: it inflates the safety bubble to maximum for
** that cycle and can erase a gap the car could have driven through. Measured
** on a real track, that happened in 9 of 254 cycles. So isolated dropouts are
** interpolated from their neighbours, and only a run of them collapses to the
** floor.
**
** Callers still detect a wholly faulty scan by noticing every ray at the floor.
*/
static void sanitise(double *ranges, const config_t *cfg)
{
    double source[N4S_LIDAR_RAYS];

    memcpy(source, ranges, sizeof(source));
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++) {
        double left = (i > 0) ? source[i - 1] : 0.0;
        double right = (i + 1 < N4S_LIDAR_RAYS) ? source[i + 1] : 0.0;

        if (!ray_is_valid(source[i])) {
            if (ray_is_valid(left) && ray_is_valid(right))
                ranges[i] = (left + right) / 2.0;
            else if (ray_is_valid(left))
                ranges[i] = left;
            else if (ray_is_valid(right))
                ranges[i] = right;
            else
                ranges[i] = cfg->lidar_floor_mm;
        }
        ranges[i] = clampd(ranges[i], cfg->lidar_floor_mm, N4S_LIDAR_MAX_MM);
    }
}

/* Centred moving average. The window shrinks at the edges rather than
** wrapping, because ray 0 and ray 31 are not neighbours in the real world. */
static void smooth(double *ranges, int window)
{
    double source[N4S_LIDAR_RAYS];
    int half = window / 2;

    if (window <= 1)
        return;
    memcpy(source, ranges, sizeof(source));
    for (int i = 0; i < (int)N4S_LIDAR_RAYS; i++) {
        double sum = 0.0;
        int count = 0;

        for (int k = i - half; k <= i + half; k++) {
            if (k < 0 || k >= (int)N4S_LIDAR_RAYS)
                continue;
            sum += source[k];
            count++;
        }
        ranges[i] = sum / (double)count;
    }
}

/*
** The summary distances the throttle law reads.
**
** forward_mm is the *minimum* over the central rays, not their mean: a thin
** obstacle dead ahead would survive averaging as a harmless-looking number,
** and we would drive into it at full power.
*/
static void summarise(scan_t *scan)
{
    size_t third = N4S_LIDAR_RAYS / 3;
    size_t nearest = argmin_range(scan->ranges, FORWARD_FIRST, FORWARD_LAST);

    scan->forward_mm = scan->ranges[nearest];
    scan->left_mm = mean_range(scan->ranges, 0, third);
    scan->right_mm = mean_range(scan->ranges, N4S_LIDAR_RAYS - third,
        N4S_LIDAR_RAYS);
    scan->closest_index = argmin_range(scan->ranges, 0, N4S_LIDAR_RAYS);
    scan->closest_mm = scan->ranges[scan->closest_index];
}

void scan_build(scan_t *scan, const double raw[N4S_LIDAR_RAYS],
    const config_t *cfg)
{
    if (scan == NULL || raw == NULL || cfg == NULL)
        return;
    memset(scan, 0, sizeof(*scan));
    normalise_order(scan->ranges, raw, cfg->mirror);
    sanitise(scan->ranges, cfg);
    smooth(scan->ranges, cfg->smooth_window);
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        scan->angles[i] = ray_angle(i, false);
    summarise(scan);
}

void scan_apply_bubble(scan_t *scan, const config_t *cfg)
{
    double distance;
    double half_angle;
    double centre;

    if (scan == NULL || cfg == NULL || cfg->bubble_mm <= 0.0)
        return;
    distance = fmax(scan->closest_mm, 1.0);
    /* Angular half-width of an object `bubble_mm` wide seen from `distance`.
    ** The closer the obstacle, the wider the slice we refuse to aim at --
    ** which is exactly the behaviour you want when hugging a corner. */
    half_angle = atan2(cfg->bubble_mm, distance) * 180.0 / M_PI;
    /* Capped so the bubble never covers more than half of the cone.
    **
    ** The exact geometry is honest but unusable at close range: an obstacle
    ** 260mm wide seen from 500mm subtends 27 degrees each side, which erases
    ** 45 of the 60 degrees we can see. The car would then find no gap in a
    ** corridor it fits through perfectly well, and report a dead end in the
    ** middle of a hairpin. Keeping half the cone alive is what makes the gap
    ** search still mean something in the situations that matter most. */
    half_angle = fmin(half_angle, N4S_LIDAR_FOV_DEG * 0.25);
    centre = scan->angles[scan->closest_index];
    /*
    ** Attenuate rather than erase.
    **
    ** A hard cut makes the forbidden sector an on/off mask centred on a single
    ** closest ray, and when two obstacles are nearly equidistant that ray
    ** flips between them, so the mask teleports and the gap search changes its
    ** mind. Fading smoothly -- nothing in the obstacle's own direction, full
    ** range at the edge -- keeps the safety property while leaving the
    ** distance field continuous for everything downstream.
    */
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++) {
        double offset = fabs(scan->angles[i] - centre) / half_angle;

        if (offset >= 1.0)
            continue;
        /* smoothstep: 0 at the obstacle, 1 at the edge, flat at both. */
        scan->ranges[i] *= offset * offset * (3.0 - 2.0 * offset);
    }
}

/*
** Score an opening by the drivable space it offers: summed squared depth past
** the threshold. Width and depth trade off in one number, and unlike "longest
** run" it changes smoothly as a wall edge crosses a ray boundary rather than
** stepping.
*/
static double run_score(const double *ranges, double threshold,
    size_t begin, size_t end)
{
    double score = 0.0;

    for (size_t i = begin; i <= end; i++) {
        double depth = ranges[i] - threshold;

        if (depth > 0.0)
            score += depth * depth;
    }
    return score;
}

/*
** Pick the opening to drive through: the best-scoring one, unless the car is
** already committed to another that is nearly as good.
**
** Switching between two near-equal openings on alternate cycles is what makes
** the heading target teleport across the cone. A rival has to beat the
** incumbent by `hysteresis` of its score to take over.
*/
static bool best_run(const double *ranges, const double *angles,
    double threshold, double previous, double hysteresis,
    size_t *begin, size_t *end)
{
    double best = 0.0;
    double incumbent = 0.0;
    size_t inc_begin = 0;
    size_t inc_end = 0;
    bool has_incumbent = false;
    size_t run_start = 0;
    bool in_run = false;
    bool found = false;

    for (size_t i = 0; i <= N4S_LIDAR_RAYS; i++) {
        bool open = (i < N4S_LIDAR_RAYS) && (ranges[i] >= threshold);

        if (open && !in_run) {
            in_run = true;
            run_start = i;
        } else if (!open && in_run) {
            size_t last = i - 1;
            double score = run_score(ranges, threshold, run_start, last);

            in_run = false;
            if (score > best) {
                best = score;
                *begin = run_start;
                *end = last;
                found = true;
            }
            if (isfinite(previous) && previous >= angles[run_start]
                && previous <= angles[last] && score > incumbent) {
                incumbent = score;
                inc_begin = run_start;
                inc_end = last;
                has_incumbent = true;
            }
        }
    }
    if (has_incumbent && incumbent >= best * (1.0 - hysteresis)) {
        *begin = inc_begin;
        *end = inc_end;
    }
    return found;
}

/*
** Where to aim inside the chosen opening: a depth-weighted centroid over its
** deepest plateau.
**
** Two mistakes are avoided here, and both were made on the way to it.
**
** Returning a ray *index* -- the middle of the plateau -- means the answer can
** only be a multiple of 1.9 degrees, and it steps whenever the plateau shifts
** by one ray. Measured on a real track: 8 to 10 degree jumps per cycle and a
** steering reversal on half of them.
**
** But a centroid over the *whole* opening is wrong in a different way: it
** splits the difference between the way through a corner and the near space
** beside it, and the car cuts the corner.
**
** Restricting the centroid to the plateau keeps the old intent -- follow the
** deep part, which is the corridor -- and makes the answer continuous.
*/
static double plateau_aim(const double *ranges, const double *angles,
    double threshold, size_t begin, size_t end)
{
    double deepest = ranges[argmax_range(ranges, begin, end + 1)];
    double floor_value = fmax(deepest - PLATEAU_TOLERANCE_MM, threshold);
    double weight_sum = 0.0;
    double angle_sum = 0.0;

    for (size_t i = begin; i <= end; i++) {
        double depth = ranges[i] - floor_value;

        if (depth <= 0.0)
            continue;
        weight_sum += depth * depth;
        angle_sum += depth * depth * angles[i];
    }
    if (weight_sum <= 0.0)
        return angles[(begin + end) / 2];
    return angle_sum / weight_sum;
}

void scan_find_gap(const scan_t *scan, const config_t *cfg, gap_t *gap)
{
    scan_find_gap_from(scan, cfg, NAN, gap);
}

void scan_find_gap_from(const scan_t *scan, const config_t *cfg,
    double previous_angle, gap_t *gap)
{
    size_t begin = 0;
    size_t end = 0;

    if (scan == NULL || cfg == NULL || gap == NULL)
        return;
    memset(gap, 0, sizeof(*gap));
    if (!best_run(scan->ranges, scan->angles, cfg->gap_threshold_mm,
        previous_angle, cfg->gap_hysteresis, &begin, &end)) {
        gap->found = false;
        return;
    }
    gap->found = true;
    gap->begin = begin;
    gap->end = end;
    gap->target_angle = plateau_aim(scan->ranges, scan->angles,
        cfg->gap_threshold_mm, begin, end);
    /* The ray the aim falls closest to, for telemetry. */
    gap->target = begin;
    for (size_t i = begin; i <= end; i++)
        if (fabs(scan->angles[i] - gap->target_angle)
            < fabs(scan->angles[gap->target] - gap->target_angle))
            gap->target = i;
    gap->depth_mm = scan->ranges[argmax_range(scan->ranges, begin, end + 1)];
}
