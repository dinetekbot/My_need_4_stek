/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** n4s.h -- the wire protocol.
**
** The car is controlled through a line-oriented text protocol spoken over
** stdin/stdout. We send one command per line, and the simulator API answers
** with exactly one response line. Everything else in this project is built
** on top of the two functions declared here: n4s_request() and
** n4s_parse_response().
*/

#ifndef N4S_H
#define N4S_H

#include <stdbool.h>
#include <stddef.h>

/* The lidar sweeps a 60 degree cone in front of the car, sampled by 32 rays.
** Distances are millimetres, saturated at 3010mm: a ray reporting 3010 means
** "nothing within reach", not "an obstacle at exactly 3010mm". */
#define N4S_LIDAR_RAYS      32
#define N4S_LIDAR_FOV_DEG   60.0
#define N4S_LIDAR_MAX_MM    3010.0

#define N4S_CODE_MAX        128
#define N4S_INFO_MAX        256
#define N4S_LINE_MAX        1024

/*
** VALUE_ID values defined by the API. We only branch on a handful of them,
** but naming all of them makes telemetry logs readable.
*/
enum n4s_value_id {
    N4S_ID_NOT_LAUNCHED     = 0,
    N4S_ID_NO_ERROR         = 1,
    N4S_ID_RUNNING          = 2,
    N4S_ID_UNDETAILED_ERROR = 3,
    N4S_ID_CHECKPOINT_ERROR = 5,
    N4S_ID_NETWORK_FAILURE  = 6,
    N4S_ID_SERVER_ERROR     = 7,
    N4S_ID_CLIENT_ERROR     = 8,
    N4S_ID_EOF              = 9,
    N4S_ID_SIM_ENDED        = 10,
    N4S_ID_EMPTY_COMMAND    = 11,
    N4S_ID_UNKNOWN_COMMAND  = 12,
    N4S_ID_WRONG_ARGS       = 13,
    N4S_ID_TOO_MANY_ARGS    = 14,
    N4S_ID_PIPELINE_FAILURE = 15,
    N4S_ID_UNEXPECTED_VALUE = 16,
    N4S_ID_CAMERA_FAILURE   = 17,
    N4S_ID_COMMAND_NOT_FOUND = 18,
    N4S_ID_ALREADY_RUNNING  = 19,
    N4S_ID_CYCLE_DONE       = 20,
    N4S_ID_SENSOR_FAILURE   = 21,
    N4S_ID_NOT_ENOUGH_CPS   = 22,
    N4S_ID_TIMER_INIT_FAIL  = 23,
    N4S_ID_TIMER_GET_FAIL   = 24,
    N4S_ID_MAP_LOAD_FAIL    = 25
};

/*
** Track progress is not reported by a dedicated command: it is piggy-backed
** on the ADDITIONAL_INFO field of *any* response. A GET_CURRENT_SPEED reply
** can be the one that tells us the race is over, so every single response
** must be inspected for it.
*/
typedef enum {
    N4S_CP_NONE = 0,
    N4S_CP_FIRST,        /* "First CP Cleared" -- first checkpoint of the track */
    N4S_CP_INTERMEDIATE, /* "CP Cleared"       -- any middle checkpoint        */
    N4S_CP_LAP,          /* "Lap Cleared"      -- one full lap completed       */
    N4S_CP_TRACK         /* "Track Cleared"    -- race over, we must stop      */
} n4s_checkpoint_t;

/*
** A parsed response line.
**
** Wire format:  VALUE_ID:STATUS:CODE_STR[:payload...]:ADDITIONAL_INFO
**
** The payload is absent, a single float, 32 floats (lidar), or a simulation
** timestamp. ADDITIONAL_INFO is free text that may itself contain ':', which
** is precisely why this parser does not simply split the line on ':'.
*/
typedef struct {
    int              value_id;
    bool             ok;                        /* STATUS field was "OK" */
    char             code[N4S_CODE_MAX];        /* CODE_STR, human readable */
    double           values[N4S_LIDAR_RAYS];    /* numeric payload, if any */
    size_t           value_count;               /* 0, 1, or N4S_LIDAR_RAYS */
    char             info[N4S_INFO_MAX];        /* ADDITIONAL_INFO verbatim */
    n4s_checkpoint_t checkpoint;                /* extracted from `info` */
    bool             has_simtime;
    long             sim_seconds;
    long             sim_nanos;
} n4s_response_t;

/*
** Parse one response line. `line` may keep its trailing newline.
** Returns 0 on success, -1 if the line is too malformed to be usable.
** On success `out` is fully initialised (no field left stale).
*/
int n4s_parse_response(const char *line, n4s_response_t *out);

/* Classify an ADDITIONAL_INFO string into a checkpoint event. */
n4s_checkpoint_t n4s_classify_checkpoint(const char *info);

/*
** Send a command and read back its response.
**
** `command` must NOT contain a trailing newline; it is appended here.
** Returns 0 on success, -1 on I/O failure or end of stream (the simulator
** went away). A non-OK response is *not* an error at this level: inspect
** out->ok and out->value_id.
*/
int n4s_request(const char *command, n4s_response_t *out);

/* Same, with a float argument formatted to 3 decimals, e.g. "CAR_FORWARD:0.500". */
int n4s_request_f(const char *command, double argument, n4s_response_t *out);

/* Typed helpers for the commands we actually use. */
int n4s_start_simulation(void);
int n4s_stop_simulation(void);
int n4s_set_throttle(double power);      /* CAR_FORWARD,   [0;1] */
int n4s_set_reverse(double power);       /* CAR_BACKWARDS, [0;1] */
int n4s_set_wheels(double direction);    /* WHEELS_DIR,    [-1;1] */
int n4s_get_lidar(double out[N4S_LIDAR_RAYS]);
int n4s_get_speed(double *out);
int n4s_get_wheels(double *out);
int n4s_get_speed_max(double *out);
int n4s_get_speed_min(double *out);

/*
** Every response passes through here, so the driving loop can learn about a
** checkpoint no matter which command triggered it. Returns the most advanced
** checkpoint seen so far and the completed lap count.
*/
void             n4s_track_observe(const n4s_response_t *response);
n4s_checkpoint_t n4s_track_last_event(void);
int              n4s_track_laps(void);
void             n4s_track_reset(void);

#endif /* N4S_H */
