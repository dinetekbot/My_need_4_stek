/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** config.c -- defaults and command line overrides.
**
** Every knob lives in one table. The parser, the --help text and the
** telemetry dump are all generated from that table, so an option cannot be
** added to one and forgotten in the others. Tuning the car is then a matter
** of restarting it with different numbers rather than editing and rebuilding,
** which is what makes a tuning session take minutes instead of an evening.
*/

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "util.h"

typedef enum {
    OPT_DOUBLE,
    OPT_INT,
    OPT_BOOL
} opt_type_t;

typedef struct {
    const char *name;
    opt_type_t  type;
    size_t      offset;
    const char *help;
} option_t;

static const option_t OPTIONS[] = {
    {"lidar-floor", OPT_DOUBLE, offsetof(config_t, lidar_floor_mm),
        "mm; distance substituted for a faulty ray"},
    {"smooth", OPT_INT, offsetof(config_t, smooth_window),
        "rays; moving average width, 1 disables"},
    {"bubble", OPT_DOUBLE, offsetof(config_t, bubble_mm),
        "mm; exclusion radius around the closest obstacle"},
    {"gap-threshold", OPT_DOUBLE, offsetof(config_t, gap_threshold_mm),
        "mm; a ray must reach this far to count as free space"},
    {"kp", OPT_DOUBLE, offsetof(config_t, steer_kp),
        "steering proportional gain"},
    {"kd", OPT_DOUBLE, offsetof(config_t, steer_kd),
        "steering derivative gain"},
    {"balance", OPT_DOUBLE, offsetof(config_t, steer_balance),
        "weight of the corridor centring term"},
    {"steer-rate", OPT_DOUBLE, offsetof(config_t, steer_rate_limit),
        "max change of WHEELS_DIR per cycle"},
    {"throttle-min", OPT_DOUBLE, offsetof(config_t, throttle_min),
        "slowest racing power, [0;1]"},
    {"throttle-max", OPT_DOUBLE, offsetof(config_t, throttle_max),
        "fastest racing power, [0;1]"},
    {"dist-slow", OPT_DOUBLE, offsetof(config_t, dist_slow_mm),
        "mm; forward distance mapped to throttle-min"},
    {"dist-fast", OPT_DOUBLE, offsetof(config_t, dist_fast_mm),
        "mm; forward distance mapped to throttle-max"},
    {"turn-penalty", OPT_DOUBLE, offsetof(config_t, turn_penalty),
        "fraction of throttle given up at full lock"},
    {"brake-dist", OPT_DOUBLE, offsetof(config_t, brake_dist_mm),
        "mm; cut the throttle below this"},
    {"reverse-dist", OPT_DOUBLE, offsetof(config_t, reverse_dist_mm),
        "mm; start backing up below this"},
    {"reverse-power", OPT_DOUBLE, offsetof(config_t, reverse_power),
        "CAR_BACKWARDS power while reversing, [0;1]"},
    {"deadend-cycles", OPT_INT, offsetof(config_t, deadend_cycles),
        "consecutive gapless cycles before declaring a dead end"},
    {"deadend-clearance", OPT_DOUBLE, offsetof(config_t, deadend_clearance_mm),
        "mm; required clearance before ending the simulation"},
    {"laps", OPT_INT, offsetof(config_t, lap_limit),
        "stop after this many laps, 0 for unlimited"},
    {"mirror", OPT_BOOL, offsetof(config_t, mirror),
        "flip the left/right convention"},
    {"verbose", OPT_BOOL, offsetof(config_t, verbose),
        "print one telemetry line per cycle on stderr"}
};

static const size_t OPTION_COUNT = sizeof(OPTIONS) / sizeof(OPTIONS[0]);

void config_defaults(config_t *cfg)
{
    if (cfg == NULL)
        return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->lidar_floor_mm = 1.0;
    cfg->smooth_window = 3;
    cfg->bubble_mm = 260.0;
    cfg->gap_threshold_mm = 1200.0;
    cfg->steer_kp = 0.85;
    cfg->steer_kd = 0.35;
    cfg->steer_balance = 0.12;
    cfg->steer_rate_limit = 0.25;
    cfg->throttle_min = 0.15;
    cfg->throttle_max = 1.0;
    cfg->dist_slow_mm = 600.0;
    cfg->dist_fast_mm = 2600.0;
    cfg->turn_penalty = 0.55;
    cfg->brake_dist_mm = 320.0;
    cfg->reverse_dist_mm = 260.0;
    cfg->reverse_power = 0.35;
    cfg->deadend_cycles = 12;
    cfg->deadend_clearance_mm = 1000.0;
    cfg->lap_limit = 0;
    cfg->mirror = false;
    cfg->verbose = false;
}

/* Address of an option's field inside a given config. */
static void *field_of(config_t *cfg, const option_t *option)
{
    return (char *)cfg + option->offset;
}

static bool assign_bool(void *field, const char *value)
{
    if (value == NULL || strcmp(value, "1") == 0
        || strcmp(value, "true") == 0 || strcmp(value, "yes") == 0) {
        *(bool *)field = true;
        return true;
    }
    if (strcmp(value, "0") == 0 || strcmp(value, "false") == 0
        || strcmp(value, "no") == 0) {
        *(bool *)field = false;
        return true;
    }
    return false;
}

static bool assign(config_t *cfg, const option_t *option, const char *value)
{
    void *field = field_of(cfg, option);
    double number;

    if (option->type == OPT_BOOL)
        return assign_bool(field, value);
    if (value == NULL || !parse_double(value, &number))
        return false;
    if (option->type == OPT_INT)
        *(int *)field = (int)number;
    else
        *(double *)field = number;
    return true;
}

void config_usage(const char *program_name)
{
    char line[256];

    snprintf(line, sizeof(line),
        "usage: %s [--option=value ...]\n\n"
        "Reads simulator responses on stdin, writes commands on stdout.\n"
        "Options:\n", program_name);
    log_line(line);
    for (size_t i = 0; i < OPTION_COUNT; i++) {
        snprintf(line, sizeof(line), "  --%-20s %s\n",
            OPTIONS[i].name, OPTIONS[i].help);
        log_line(line);
    }
    log_line("  --help                 show this message\n");
}

int config_parse_args(config_t *cfg, int argc, char **argv)
{
    char name[64];
    char message[256];

    for (int i = 1; i < argc; i++) {
        const char *argument = argv[i];
        const char *equals;
        const char *value = NULL;
        size_t length;
        bool matched = false;

        if (strncmp(argument, "--", 2) != 0) {
            snprintf(message, sizeof(message),
                "error: unexpected argument '%s'\n", argument);
            log_line(message);
            return -1;
        }
        argument += 2;
        if (strcmp(argument, "help") == 0)
            return 1;
        equals = strchr(argument, '=');
        length = (equals != NULL) ? (size_t)(equals - argument)
            : strlen(argument);
        if (length >= sizeof(name))
            length = sizeof(name) - 1;
        memcpy(name, argument, length);
        name[length] = '\0';
        if (equals != NULL)
            value = equals + 1;
        for (size_t k = 0; k < OPTION_COUNT && !matched; k++) {
            if (strcmp(name, OPTIONS[k].name) != 0)
                continue;
            matched = true;
            if (!assign(cfg, &OPTIONS[k], value)) {
                snprintf(message, sizeof(message),
                    "error: bad value for --%s\n", name);
                log_line(message);
                return -1;
            }
        }
        if (!matched) {
            snprintf(message, sizeof(message),
                "error: unknown option --%s\n", name);
            log_line(message);
            return -1;
        }
    }
    return 0;
}

void config_dump(const config_t *cfg)
{
    char line[128];

    if (cfg == NULL)
        return;
    log_line("[config]\n");
    for (size_t i = 0; i < OPTION_COUNT; i++) {
        const void *field = (const char *)cfg + OPTIONS[i].offset;

        if (OPTIONS[i].type == OPT_BOOL)
            snprintf(line, sizeof(line), "  %-20s %s\n", OPTIONS[i].name,
                *(const bool *)field ? "true" : "false");
        else if (OPTIONS[i].type == OPT_INT)
            snprintf(line, sizeof(line), "  %-20s %d\n", OPTIONS[i].name,
                *(const int *)field);
        else
            snprintf(line, sizeof(line), "  %-20s %.3f\n", OPTIONS[i].name,
                *(const double *)field);
        log_line(line);
    }
}
