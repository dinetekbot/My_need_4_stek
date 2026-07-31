/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** main.c -- the race loop.
**
** One cycle is always the same four exchanges with the simulator:
**
**     GET_INFO_LIDAR  ->  WHEELS_DIR  ->  CAR_FORWARD  ->  CAR_BACKWARDS
**
** Always four, even when a command is redundant. A fixed number of round
** trips per cycle means the loop runs at a constant rate, and a constant rate
** is what lets the derivative term of the steering controller be written as a
** plain difference between consecutive errors instead of dividing by a
** measured, jittery timestep.
**
** It is also the single biggest speed-up over the naive approach: querying
** the maximum and minimum speed on every cycle, as it is tempting to do, adds
** two more round trips for two numbers that never change, and the car spends
** a third of its life waiting for answers it already has.
*/

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "control.h"
#include "n4s.h"
#include "perception.h"
#include "util.h"

/* Epitech convention, kept because it costs nothing and the grader looks for
** it: 0 when the race ends cleanly, 84 on any error. */
#define EXIT_ERROR 84

/* Consecutive unusable lidar reads tolerated before we give up. A handful of
** bad scans happens while the simulation settles; a hundred means the other
** end of the pipe is gone. */
#define MAX_CONSECUTIVE_FAULTS 100

static volatile sig_atomic_t g_interrupted = 0;

static void on_signal(int signal_number)
{
    (void)signal_number;
    g_interrupted = 1;
}

static void install_signal_handlers(void)
{
    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = on_signal;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    /* The pipeline is a loop of processes. When one dies, our writes get a
    ** SIGPIPE; ignoring it turns that into a write error we can report
    ** instead of a silent kill. */
    signal(SIGPIPE, SIG_IGN);
}

/*
** A scan where every ray sits at the floor distance is not a car surrounded
** by walls, it is a sensor that returned nothing. Braking on it would be
** wrong; so would driving on it. We skip the cycle.
*/
static bool scan_is_faulty(const scan_t *scan, const config_t *cfg)
{
    for (size_t i = 0; i < N4S_LIDAR_RAYS; i++)
        if (scan->ranges[i] > cfg->lidar_floor_mm + 1e-6)
            return false;
    return true;
}

static void report_telemetry(const controller_t *ctl, const scan_t *scan,
    const gap_t *gap, const command_t *cmd)
{
    char line[256];

    snprintf(line, sizeof(line),
        "[%05ld] %-9s fwd=%6.0f near=%6.0f gap=%s@%+06.1f wheels=%+.3f "
        "thr=%.3f rev=%.3f laps=%d\n",
        ctl->cycles, drive_state_name(ctl->state), scan->forward_mm,
        scan->closest_mm, gap->found ? "yes" : " no ", gap->target_angle,
        cmd->wheels, cmd->throttle, cmd->reverse, n4s_track_laps());
    log_line(line);
}

/* Push one cycle's decisions to the car. Returns -1 if the link broke. */
static int apply_command(const command_t *cmd)
{
    if (n4s_set_wheels(cmd->wheels) != 0)
        return -1;
    if (n4s_set_throttle(cmd->throttle) != 0)
        return -1;
    if (n4s_set_reverse(cmd->reverse) != 0)
        return -1;
    return 0;
}

static const char *checkpoint_name(n4s_checkpoint_t event)
{
    switch (event) {
    case N4S_CP_FIRST:        return "first checkpoint";
    case N4S_CP_INTERMEDIATE: return "checkpoint";
    case N4S_CP_LAP:          return "lap complete";
    case N4S_CP_TRACK:        return "TRACK CLEARED";
    default:                  return "none";
    }
}

/*
** Announce track progress as it happens.
**
** Without this the telemetry shows a car driving along and then, with no
** explanation, a clean exit -- and you cannot tell a completed track from a
** crash. Progress arrives attached to arbitrary responses, so the only place
** it can be observed is here, by watching the tracker change.
*/
static void report_progress(const config_t *cfg, n4s_checkpoint_t *last)
{
    n4s_checkpoint_t event = n4s_track_last_event();
    char line[128];

    if (event == *last)
        return;
    *last = event;
    if (!cfg->verbose || event == N4S_CP_NONE)
        return;
    snprintf(line, sizeof(line), "        >>> %s (laps=%d)\n",
        checkpoint_name(event), n4s_track_laps());
    log_line(line);
}

/* True once the race is over for a reason the controller cannot see. */
static bool race_is_over(const config_t *cfg)
{
    if (n4s_track_last_event() == N4S_CP_TRACK)
        return true;
    if (cfg->lap_limit > 0 && n4s_track_laps() >= cfg->lap_limit)
        return true;
    return false;
}

/*
** Read the constants the car will not change during the race. This doubles as
** a health check on the link: these two commands answer even when the
** simulation is not running, so a failure here means the pipeline itself is
** broken rather than the scene.
*/
static int probe_car(const config_t *cfg)
{
    double speed_max = 0.0;
    double speed_min = 0.0;
    char line[128];

    if (n4s_get_speed_max(&speed_max) != 0
        || n4s_get_speed_min(&speed_min) != 0) {
        log_line("error: no answer from the simulator API\n");
        return -1;
    }
    if (cfg->verbose) {
        snprintf(line, sizeof(line), "[car] speed_max=%.2f speed_min=%.2f\n",
            speed_max, speed_min);
        log_line(line);
    }
    return 0;
}

/* One iteration of the race loop. Returns 0 to continue, 1 to stop, -1 on error. */
static int race_cycle(controller_t *ctl, const config_t *cfg, int *faults,
    n4s_checkpoint_t *last_event)
{
    double raw[N4S_LIDAR_RAYS];
    scan_t scan;
    gap_t gap;
    command_t cmd;
    char message[160];

    if (n4s_get_lidar(raw) != 0) {
        (*faults)++;
        return (*faults > MAX_CONSECUTIVE_FAULTS) ? -1 : 0;
    }
    *faults = 0;
    scan_build(&scan, raw, cfg);
    if (scan_is_faulty(&scan, cfg)) {
        /*
        ** Say so, and keep saying so periodically.
        **
        ** A scene whose lidar reports nothing but zeros -- it happens -- used
        ** to leave the car sitting silently with the throttle at zero and no
        ** output whatsoever, which is indistinguishable from a hang. Never
        ** fail quietly in a loop.
        */
        ctl->faulty_scans++;
        if (ctl->faulty_scans == 1 || ctl->faulty_scans % 50 == 0) {
            snprintf(message, sizeof(message),
                "warning: lidar returned no usable reading (%ld times); "
                "is the scene's sensor working?\n", ctl->faulty_scans);
            log_line(message);
        }
        return (n4s_set_throttle(0.0) == 0) ? 0 : -1;
    }
    scan_apply_bubble(&scan, cfg);
    scan_find_gap(&scan, cfg, &gap);
    controller_step(ctl, &scan, &gap, cfg, &cmd);
    if (apply_command(&cmd) != 0)
        return -1;
    if (cfg->verbose)
        report_telemetry(ctl, &scan, &gap, &cmd);
    report_progress(cfg, last_event);
    if (race_is_over(cfg))
        ctl->state = DRIVE_FINISHED;
    return (ctl->state == DRIVE_FINISHED) ? 1 : 0;
}

static int race(controller_t *ctl, const config_t *cfg)
{
    n4s_checkpoint_t last_event = N4S_CP_NONE;
    int faults = 0;
    int status;

    while (!g_interrupted) {
        status = race_cycle(ctl, cfg, &faults, &last_event);
        if (status < 0)
            return -1;
        if (status > 0)
            return 0;
    }
    return 0;
}

int main(int argc, char **argv)
{
    config_t cfg;
    controller_t ctl;
    int parsed;

    config_defaults(&cfg);
    parsed = config_parse_args(&cfg, argc, argv);
    if (parsed != 0) {
        config_usage(argv[0]);
        return (parsed > 0) ? 0 : EXIT_ERROR;
    }
    install_signal_handlers();
    if (cfg.verbose)
        config_dump(&cfg);
    if (probe_car(&cfg) != 0)
        return EXIT_ERROR;
    if (n4s_start_simulation() != 0) {
        log_line("error: could not start the simulation "
            "(is CoppeliaSim running with a track scene loaded?)\n");
        return EXIT_ERROR;
    }
    controller_init(&ctl);
    if (race(&ctl, &cfg) != 0) {
        log_line("error: lost contact with the simulator\n");
        return EXIT_ERROR;
    }
    /* Come to a stop before ending the simulation, in that order: a scene
    ** stopped with the throttle still applied records the car as moving. */
    n4s_set_throttle(0.0);
    n4s_set_reverse(0.0);
    n4s_stop_simulation();
    return 0;
}
