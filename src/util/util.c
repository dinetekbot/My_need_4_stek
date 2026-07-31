/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** util.c -- small numeric helpers.
*/

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "util.h"

double clampd(double value, double low, double high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

double map_range(double x, double in_low, double in_high,
    double out_low, double out_high)
{
    double span = in_high - in_low;
    double ratio;

    /* A degenerate input range would divide by zero. Returning the low end
    ** is arbitrary but predictable, which is what matters in a control loop. */
    if (fabs(span) < 1e-9)
        return out_low;
    ratio = (x - in_low) / span;
    ratio = clampd(ratio, 0.0, 1.0);
    return out_low + ratio * (out_high - out_low);
}

double mean_range(const double *values, size_t from, size_t to)
{
    double sum = 0.0;

    if (values == NULL || to <= from)
        return 0.0;
    for (size_t i = from; i < to; i++)
        sum += values[i];
    return sum / (double)(to - from);
}

size_t argmin_range(const double *values, size_t from, size_t to)
{
    size_t best = from;

    if (values == NULL || to <= from)
        return from;
    for (size_t i = from; i < to; i++)
        if (values[i] < values[best])
            best = i;
    return best;
}

size_t argmax_range(const double *values, size_t from, size_t to)
{
    size_t best = from;

    if (values == NULL || to <= from)
        return from;
    for (size_t i = from; i < to; i++)
        if (values[i] > values[best])
            best = i;
    return best;
}

double rate_limit(double target, double previous, double max_delta)
{
    double delta = target - previous;

    if (max_delta <= 0.0)
        return target;
    if (delta > max_delta)
        return previous + max_delta;
    if (delta < -max_delta)
        return previous - max_delta;
    return target;
}

bool parse_double(const char *text, double *out)
{
    char *end = NULL;
    double value;

    if (text == NULL || *text == '\0')
        return false;
    errno = 0;
    value = strtod(text, &end);
    if (end == text || errno == ERANGE)
        return false;
    /* Trailing whitespace is fine, trailing anything else is not: we use this
    ** to decide whether a protocol field is a number or free text. */
    while (*end != '\0' && isspace((unsigned char)*end))
        end++;
    if (*end != '\0')
        return false;
    if (!isfinite(value))
        return false;
    if (out != NULL)
        *out = value;
    return true;
}

void log_line(const char *text)
{
    size_t len;
    ssize_t written;
    size_t done = 0;

    if (text == NULL)
        return;
    len = strlen(text);
    while (done < len) {
        written = write(STDERR_FILENO, text + done, len - done);
        if (written <= 0)
            return;
        done += (size_t)written;
    }
}
