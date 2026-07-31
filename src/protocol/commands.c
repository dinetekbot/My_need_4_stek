/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** commands.c -- typed wrappers around the raw text commands, plus the race
** progress tracker.
**
** Wrapping each command in a function that clamps its argument means an
** out-of-range value can never reach the wire. The API answers an
** out-of-range argument with an error and leaves the actuator untouched, so
** an unclamped throttle does not go faster -- it does nothing at all, which
** looks exactly like a car that mysteriously stops accelerating.
*/

#include <string.h>

#include "n4s.h"
#include "util.h"

/* ------------------------------------------------------------------ */
/* Race progress                                                       */
/* ------------------------------------------------------------------ */

static n4s_checkpoint_t g_last_event = N4S_CP_NONE;
static int g_laps = 0;

void n4s_track_observe(const n4s_response_t *response)
{
    if (response == NULL || response->checkpoint == N4S_CP_NONE)
        return;
    g_last_event = response->checkpoint;
    if (response->checkpoint == N4S_CP_LAP)
        g_laps++;
}

n4s_checkpoint_t n4s_track_last_event(void)
{
    return g_last_event;
}

int n4s_track_laps(void)
{
    return g_laps;
}

void n4s_track_reset(void)
{
    g_last_event = N4S_CP_NONE;
    g_laps = 0;
}

/* ------------------------------------------------------------------ */
/* Commands                                                            */
/* ------------------------------------------------------------------ */

int n4s_start_simulation(void)
{
    n4s_response_t response;

    if (n4s_request("START_SIMULATION", &response) != 0)
        return -1;
    /* Being already up is a success from our point of view: it happens when
    ** the simulation was started by hand before the AI was launched. */
    if (!response.ok && response.value_id != N4S_ID_ALREADY_RUNNING)
        return -1;
    return 0;
}

int n4s_stop_simulation(void)
{
    n4s_response_t response;

    if (n4s_request("STOP_SIMULATION", &response) != 0)
        return -1;
    return 0;
}

int n4s_set_throttle(double power)
{
    return n4s_request_f("CAR_FORWARD", clampd(power, 0.0, 1.0), NULL);
}

int n4s_set_reverse(double power)
{
    return n4s_request_f("CAR_BACKWARDS", clampd(power, 0.0, 1.0), NULL);
}

int n4s_set_wheels(double direction)
{
    return n4s_request_f("WHEELS_DIR", clampd(direction, -1.0, 1.0), NULL);
}

int n4s_get_lidar(double out[N4S_LIDAR_RAYS])
{
    n4s_response_t response;

    if (out == NULL)
        return -1;
    if (n4s_request("GET_INFO_LIDAR", &response) != 0)
        return -1;
    /* A short scan means the sensor hiccuped or the simulation is not running
    ** yet. Driving on a partial scan is worse than skipping the cycle. */
    if (!response.ok || response.value_count != N4S_LIDAR_RAYS)
        return -1;
    memcpy(out, response.values, sizeof(double) * N4S_LIDAR_RAYS);
    return 0;
}

/* All the single-value getters share this shape. */
static int get_scalar(const char *command, double *out)
{
    n4s_response_t response;

    if (out == NULL)
        return -1;
    if (n4s_request(command, &response) != 0)
        return -1;
    if (!response.ok || response.value_count < 1)
        return -1;
    *out = response.values[0];
    return 0;
}

int n4s_get_speed(double *out)
{
    return get_scalar("GET_CURRENT_SPEED", out);
}

int n4s_get_wheels(double *out)
{
    return get_scalar("GET_CURRENT_WHEELS", out);
}

int n4s_get_speed_max(double *out)
{
    return get_scalar("GET_CAR_SPEED_MAX", out);
}

int n4s_get_speed_min(double *out)
{
    return get_scalar("GET_CAR_SPEED_MIN", out);
}
