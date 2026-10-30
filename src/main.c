/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** main.c -- the race loop.
**
** Three exchanges per cycle, every cycle:
**
**     GET_INFO_LIDAR -> CAR_FORWARD -> WHEELS_DIR
**
** That is the whole conversation. The old version spent four round trips a
** cycle and carried a planner, a state machine and a safety bubble. This one
** spends three and decides with two threshold tables. Against the simulator
** it is not close: the simple version gets round the track, the clever one
** kept finding new ways to get stuck.
*/

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car.h"
#include "n4s.h"
#include "util.h"

/* Epitech convention: 0 for a clean race, 84 for any error. */
#define EXIT_ERROR 84

/* A handful of bad reads happens while the simulation settles. A hundred in
** a row means the other end of the pipe is gone. */
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
    /* The pipeline is a ring of processes. When one dies our writes get a
    ** SIGPIPE; ignoring it turns that into a write error we can report. */
    signal(SIGPIPE, SIG_IGN);
}

static bool race_is_over(void)
{
    return n4s_track_last_event() == N4S_CP_TRACK;
}

static void report(bool verbose, long cycle, const double raw[N4S_LIDAR_RAYS],
    const car_command_t *cmd)
{
    char line[128];

    if (!verbose)
        return;
    snprintf(line, sizeof(line),
        "[%05ld] fwd=%6.0f left=%6.0f right=%6.0f wheels=%+.3f thr=%.3f\n",
        cycle, raw[CAR_FORWARD_RAY], raw[CAR_LEFT_RAY], raw[CAR_RIGHT_RAY],
        cmd->wheels, cmd->throttle);
    log_line(line);
}

/*
** Push one cycle to the car. Returns 0 to continue, 1 when the track has been
** cleared, -1 when the link broke.
**
** Order matters slightly: power first, steering second, because that is the
** order the API's own examples use. Either way the simulator applies both
** before the next physics step, but keeping the familiar order removes one
** thing to wonder about.
*/
static int apply(const car_command_t *cmd)
{
    if (n4s_set_throttle(cmd->throttle) != 0)
        return -1;
    if (race_is_over())
        return 1;
    if (n4s_set_wheels(cmd->wheels) != 0)
        return -1;
    if (race_is_over())
        return 1;
    return 0;
}

static int race(bool verbose)
{
    double raw[N4S_LIDAR_RAYS];
    car_command_t cmd;
    int faults = 0;
    long cycle = 0;
    int status;

    while (!g_interrupted) {
        if (n4s_get_lidar(raw) != 0) {
            if (++faults > MAX_CONSECUTIVE_FAULTS) {
                log_line("error: too many bad lidar reads\n");
                return -1;
            }
            /* Do not keep the throttle open on a scan we could not read. */
            if (n4s_set_throttle(0.0) != 0)
                return -1;
            continue;
        }
        faults = 0;
        if (race_is_over())
            break;
        cycle++;
        car_step(raw, &cmd);
        status = apply(&cmd);
        if (status < 0)
            return -1;
        report(verbose, cycle, raw, &cmd);
        if (status > 0)
            break;
    }
    return 0;
}

static void usage(const char *program)
{
    char line[160];

    snprintf(line, sizeof(line),
        "usage: %s [--verbose]\n\n"
        "Reads simulator responses on stdin, writes commands on stdout.\n",
        program);
    log_line(line);
}

/*
** Returns 0 to run, 1 when --help was printed, -1 on a bad argument.
*/
static int parse_args(int argc, char **argv, bool *verbose)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0) {
            *verbose = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 1;
        } else {
            char line[160];

            snprintf(line, sizeof(line), "error: unknown option '%s'\n",
                argv[i]);
            log_line(line);
            usage(argv[0]);
            return -1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    n4s_response_t start;
    bool verbose = false;
    char banner[256];
    int parsed;

    parsed = parse_args(argc, argv, &verbose);
    if (parsed != 0)
        return (parsed > 0) ? 0 : EXIT_ERROR;
    install_signal_handlers();
    /*
    ** Starting costs a round trip or two, but it is worth it: STOP first means
    ** every race starts from the starting line instead of wherever a previous
    ** killed run left the car. A car that begins nose-first against a wall
    ** looks exactly like an AI driving at random.
    */
    if (n4s_start_simulation(&start) != 0) {
        snprintf(banner, sizeof(banner),
            "error: could not start the simulation -- the API said: "
            "%d:%s:%s\n", start.value_id, start.ok ? "OK" : "KO", start.code);
        log_line(banner);
        log_line("       check that CoppeliaSim has a valid track scene "
            "loaded and its simulation stopped.\n");
        return EXIT_ERROR;
    }
    if (race(verbose) != 0) {
        log_line("error: lost contact with the simulator\n");
        n4s_set_throttle(0.0);
        return EXIT_ERROR;
    }
    /* Stop in this order: a scene stopped with the throttle still applied
    ** records the car as moving. */
    n4s_set_throttle(0.0);
    n4s_set_reverse(0.0);
    n4s_stop_simulation();
    return 0;
}
