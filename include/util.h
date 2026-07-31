/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** util.h -- small numeric helpers shared by the perception and control code.
*/

#ifndef UTIL_H
#define UTIL_H

#include <stdbool.h>
#include <stddef.h>

/* Clamp `value` into [low; high]. */
double clampd(double value, double low, double high);

/*
** Linear interpolation between two ranges, clamped to the output range.
**
** map_range(x, 500, 2500, 0.2, 1.0) reads as: "when x is 500 give 0.2, when
** x is 2500 give 1.0, interpolate in between, never leave [0.2; 1.0]".
**
** The original implementation of this project had an unclamped version of
** this function, which happily returned throttle values above 1.0 and made
** the car behave unpredictably. Clamping is not a detail here.
*/
double map_range(double x, double in_low, double in_high,
    double out_low, double out_high);

/* Arithmetic mean of values[from; to[ . Returns 0 for an empty range. */
double mean_range(const double *values, size_t from, size_t to);

/* Index of the smallest / largest value in values[from; to[ . */
size_t argmin_range(const double *values, size_t from, size_t to);
size_t argmax_range(const double *values, size_t from, size_t to);

/*
** Limit how much `target` may differ from `previous` in a single step.
** Used to keep the steering from snapping between extremes, which upsets
** the physics engine and makes the car spin out.
*/
double rate_limit(double target, double previous, double max_delta);

/* Parse a double from `text`. Returns false if the text is not fully numeric. */
bool parse_double(const char *text, double *out);

/* Write a NUL-terminated string to stderr. Never touches stdout. */
void log_line(const char *text);

#endif /* UTIL_H */
