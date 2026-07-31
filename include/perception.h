/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** perception.h -- turning 32 raw distances into a decision about where the
** free space is.
**
** This is the "follow the gap" method: clean the scan, erase the area around
** the nearest obstacle, then aim at the middle of the widest remaining
** opening. It is a small amount of arithmetic with a strong guarantee -- the
** car is never steered into the closest thing it can see.
*/

#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <stdbool.h>
#include <stddef.h>

#include "config.h"
#include "n4s.h"

/* A processed scan: distances plus the geometry needed to reason about them. */
typedef struct {
    double ranges[N4S_LIDAR_RAYS];  /* cleaned distances, millimetres      */
    double angles[N4S_LIDAR_RAYS];  /* bearing of each ray, degrees        */
    double forward_mm;              /* distance straight ahead             */
    double left_mm;                 /* mean distance over the left third   */
    double right_mm;                /* mean distance over the right third  */
    double closest_mm;              /* nearest obstacle anywhere in the cone */
    size_t closest_index;
} scan_t;

/* The opening the car should drive through. */
typedef struct {
    bool   found;        /* false when the whole cone is blocked */
    size_t begin;        /* first ray of the gap, inclusive      */
    size_t end;          /* last ray of the gap, inclusive       */
    size_t target;       /* the ray we aim at                    */
    double target_angle; /* bearing of that ray, degrees         */
    double depth_mm;     /* how far the gap reaches              */
} gap_t;

/*
** Bearing of ray `index`, in degrees, negative to the left.
** The rays are the centres of 32 equal slices of the 60 degree cone, so ray 0
** sits at -30 + (60/32)/2 = -29.06 degrees, not at -30.
*/
double ray_angle(size_t index, bool inverted);

/*
** Build a scan from a raw lidar reading: reverse it if needed, reject
** nonsense values, saturate at the sensor maximum, smooth, and compute the
** summary distances. `raw` is left untouched.
*/
void scan_build(scan_t *scan, const double raw[N4S_LIDAR_RAYS],
    const config_t *cfg);

/*
** Zero out every ray within `bubble_mm` of the closest obstacle, measured as
** an arc at that obstacle's distance. Closer obstacles produce wider bubbles,
** which is what makes the car swing wide around a corner it is about to clip.
*/
void scan_apply_bubble(scan_t *scan, const config_t *cfg);

/*
** Find the widest run of rays reaching at least cfg->gap_threshold_mm, then
** aim at the middle of that run's deepest plateau. Aiming at the plateau
** rather than at the single furthest ray keeps the target from flickering
** between two nearly equal rays on every cycle.
*/
void scan_find_gap(const scan_t *scan, const config_t *cfg, gap_t *gap);

#endif /* PERCEPTION_H */
