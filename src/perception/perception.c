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
static const double PLATEAU_TOLERANCE_MM = 250.0;

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
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        if (fabs(scan->angles[i] - centre) <= half_angle)
            scan->ranges[i] = 0.0;
}

/* Longest run of rays reaching at least `threshold`. Returns false if none. */
static bool longest_run(const double *ranges, double threshold,
    size_t *begin, size_t *end)
{
    size_t best_len = 0;
    size_t run_start = 0;
    bool in_run = false;

    for (size_t i = 0; i <= N4S_LIDAR_RAYS; i++) {
        bool open = (i < N4S_LIDAR_RAYS) && (ranges[i] >= threshold);

        if (open && !in_run) {
            in_run = true;
            run_start = i;
        } else if (!open && in_run) {
            in_run = false;
            if (i - run_start > best_len) {
                best_len = i - run_start;
                *begin = run_start;
                *end = i - 1;
            }
        }
    }
    return best_len > 0;
}

/*
** Within the chosen gap, aim at the middle of the deepest plateau: the
** longest stretch of rays that all reach within PLATEAU_TOLERANCE_MM of the
** gap's deepest ray.
*/
static size_t plateau_centre(const double *ranges, size_t begin, size_t end)
{
    double deepest = ranges[argmax_range(ranges, begin, end + 1)];
    double floor_value = deepest - PLATEAU_TOLERANCE_MM;
    size_t best_begin = begin;
    size_t best_end = begin;
    size_t run_start = begin;
    bool in_run = false;

    for (size_t i = begin; i <= end + 1; i++) {
        bool high = (i <= end) && (ranges[i] >= floor_value);

        if (high && !in_run) {
            in_run = true;
            run_start = i;
        } else if (!high && in_run) {
            in_run = false;
            if (i - run_start > best_end - best_begin + 1) {
                best_begin = run_start;
                best_end = i - 1;
            }
        }
    }
    return (best_begin + best_end) / 2;
}

void scan_find_gap(const scan_t *scan, const config_t *cfg, gap_t *gap)
{
    size_t begin = 0;
    size_t end = 0;

    if (scan == NULL || cfg == NULL || gap == NULL)
        return;
    memset(gap, 0, sizeof(*gap));
    if (!longest_run(scan->ranges, cfg->gap_threshold_mm, &begin, &end)) {
        gap->found = false;
        return;
    }
    gap->found = true;
    gap->begin = begin;
    gap->end = end;
    gap->target = plateau_centre(scan->ranges, begin, end);
    gap->target_angle = scan->angles[gap->target];
    gap->depth_mm = scan->ranges[argmax_range(scan->ranges, begin, end + 1)];
}
