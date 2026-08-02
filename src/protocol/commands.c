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

/* Attempts before giving up on starting. A scene that has just been loaded,
** or a previous run that was killed mid-race, can leave the simulator
** refusing to start for a moment and then working normally. */
#define START_ATTEMPTS 5

int n4s_start_simulation(n4s_response_t *out)
{
    n4s_response_t response;

    /*
    ** Stop whatever is running before starting, always.
    **
    ** A simulation already in progress is not something to join. Joining it
    ** means the race begins wherever the car happens to have been left --
    ** and it is left somewhere awkward precisely when the previous run was
    ** killed rather than ended, which is the common case. Observed: a car
    ** starting its race nose-first against a wall with 725mm of clearance
    ** instead of the 3007mm of the starting line, immediately and correctly
    ** concluding it was stuck, and spending the whole race doing three-point
    ** turns. From the outside that looks like an AI driving at random.
    **
    ** Stopping first costs one command and makes every race start from the
    ** line. A refusal here is fine and expected when nothing was running.
    */
    n4s_request("STOP_SIMULATION", NULL);
    for (int attempt = 0; attempt < START_ATTEMPTS; attempt++) {
        if (n4s_request("START_SIMULATION", &response) != 0)
            return -1;
        if (out != NULL)
            *out = response;
        if (response.ok || response.value_id == N4S_ID_ALREADY_RUNNING)
            return 0;
        /* A run that was interrupted can leave the previous simulation
        ** going. Clearing it is worth one try before reporting failure. */
        if (n4s_request("STOP_SIMULATION", NULL) != 0)
            return -1;
        /* Let the simulator settle. CYCLE_WAIT costs one round trip and is
        ** paced by the physics engine, which is exactly the clock we want. */
        if (n4s_request("CYCLE_WAIT:5", NULL) != 0)
            return -1;
    }
    return -1;
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

int n4s_get_simtime(double *seconds)
{
    n4s_response_t response;

    if (seconds == NULL)
        return -1;
    if (n4s_request("GET_INFO_SIMTIME", &response) != 0)
        return -1;
    if (!response.ok || !response.has_simtime)
        return -1;
    *seconds = (double)response.sim_seconds
        + (double)response.sim_nanos / 1e9;
    return 0;
}
