/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_config.c -- option parsing, from the command line and the environment.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "test.h"

static void test_command_line(void)
{
    config_t cfg;
    char *argv[] = {"ai", "--kp=2.5", "--laps=3", "--mirror=0"};

    config_defaults(&cfg);
    CHECK(config_parse_args(&cfg, 4, argv) == 0);
    CHECK_NEAR(cfg.steer_kp, 2.5, 1e-9);
    CHECK(cfg.lap_limit == 3);
    CHECK(!cfg.mirror);
}

static void test_bad_options_are_refused(void)
{
    config_t cfg;
    char *unknown[] = {"ai", "--nonsense=1"};
    char *bad_value[] = {"ai", "--kp=abc"};
    char *not_an_option[] = {"ai", "kp=1"};
    char *help[] = {"ai", "--help"};

    config_defaults(&cfg);
    CHECK(config_parse_args(&cfg, 2, unknown) == -1);
    CHECK(config_parse_args(&cfg, 2, bad_value) == -1);
    CHECK(config_parse_args(&cfg, 2, not_an_option) == -1);
    CHECK(config_parse_args(&cfg, 2, help) == 1);
}

/*
** The launcher shipped with the simulator API runs the binary with no
** arguments and offers no way to add any. Without an environment channel,
** using the documented workflow would mean no telemetry and no tuning.
*/
static void test_environment(void)
{
    config_t cfg;

    setenv("TEST_AI_OPTS", "--verbose --gap-threshold=650  --laps=2", 1);
    config_defaults(&cfg);
    CHECK(config_parse_env(&cfg, "TEST_AI_OPTS") == 0);
    CHECK(cfg.verbose);
    CHECK_NEAR(cfg.gap_threshold_mm, 650.0, 1e-9);
    CHECK(cfg.lap_limit == 2);

    /* An unset or empty variable is not an error, it is the normal case. */
    unsetenv("TEST_AI_OPTS");
    config_defaults(&cfg);
    CHECK(config_parse_env(&cfg, "TEST_AI_OPTS") == 0);
    CHECK(!cfg.verbose);
    CHECK(config_parse_env(&cfg, NULL) == 0);

    setenv("TEST_AI_OPTS", "   ", 1);
    CHECK(config_parse_env(&cfg, "TEST_AI_OPTS") == 0);

    /* A bad option in the environment is still a bad option. */
    setenv("TEST_AI_OPTS", "--nonsense", 1);
    CHECK(config_parse_env(&cfg, "TEST_AI_OPTS") == -1);
    unsetenv("TEST_AI_OPTS");
}

/* Parsing the environment must not modify it: some libcs hand back a pointer
** straight into environ, and strtok would carve it up in place. */
static void test_environment_is_not_modified(void)
{
    config_t cfg;
    const char *original = "--verbose --kp=1.9";
    const char *after;

    setenv("TEST_AI_OPTS", original, 1);
    config_defaults(&cfg);
    CHECK(config_parse_env(&cfg, "TEST_AI_OPTS") == 0);
    after = getenv("TEST_AI_OPTS");
    CHECK_STR(after, original);
    unsetenv("TEST_AI_OPTS");
}

void test_config(void)
{
    printf("config\n");
    test_command_line();
    test_bad_options_are_refused();
    test_environment();
    test_environment_is_not_modified();
}
