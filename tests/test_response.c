/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test_response.c -- the protocol parser.
**
** Several of these lines are captured verbatim from a real session. They are
** here because each one broke an earlier version of this parser.
*/

#include <stdio.h>
#include <string.h>

#include "n4s.h"
#include "test.h"

static void test_simple_scalar(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response("1:OK:No errors so far:50.3:No further info",
        &r) == 0);
    CHECK(r.value_id == 1);
    CHECK(r.ok);
    CHECK_STR(r.code, "No errors so far");
    CHECK(r.value_count == 1);
    CHECK_NEAR(r.values[0], 50.3, 1e-9);
    CHECK_STR(r.info, "No further info");
    CHECK(r.checkpoint == N4S_CP_NONE);
}

static void test_error_response(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response("18:KO:Command not found:No further info",
        &r) == 0);
    CHECK(r.value_id == N4S_ID_COMMAND_NOT_FOUND);
    CHECK(!r.ok);
    CHECK(r.value_count == 0);
    CHECK_STR(r.info, "No further info");
}

static void test_no_payload_no_info(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response("2:OK:Simulation running", &r) == 0);
    CHECK(r.value_id == N4S_ID_RUNNING);
    CHECK(r.ok);
    CHECK(r.value_count == 0);
    CHECK_STR(r.info, "");
}

static void test_lidar_payload(void)
{
    char line[N4S_LINE_MAX];
    n4s_response_t r;
    size_t used;

    used = (size_t)snprintf(line, sizeof(line), "1:OK:No errors so far");
    for (int i = 0; i < N4S_LIDAR_RAYS; i++)
        used += (size_t)snprintf(line + used, sizeof(line) - used, ":%d.0",
            100 + i);
    snprintf(line + used, sizeof(line) - used, ":No further info");

    CHECK(n4s_parse_response(line, &r) == 0);
    CHECK(r.value_count == N4S_LIDAR_RAYS);
    CHECK_NEAR(r.values[0], 100.0, 1e-9);
    CHECK_NEAR(r.values[N4S_LIDAR_RAYS - 1], 131.0, 1e-9);
    CHECK_STR(r.info, "No further info");
}

/*
** The regression that motivated rewriting this parser.
**
** ADDITIONAL_INFO contains colons. Splitting the line on ':' and reading
** field 4 yields "First CP Cleared" with the timestamp stranded in field 5,
** so a checkpoint search that expects the whole string finds nothing and the
** AI silently never learns it is making progress.
*/
static void test_info_containing_colons(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response(
        "1:OK:No errors so far:-25.1:First CP Cleared: [0][88s 26029102ns]",
        &r) == 0);
    CHECK(r.value_count == 1);
    CHECK_NEAR(r.values[0], -25.1, 1e-9);
    CHECK_STR(r.info, "First CP Cleared: [0][88s 26029102ns]");
    CHECK(r.checkpoint == N4S_CP_FIRST);
}

static void test_simtime_payload(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response(
        "1:OK:No errors so far:[115s,559291238ns]:"
        "CP Cleared: [1][113s 756435254ns]", &r) == 0);
    CHECK(r.has_simtime);
    CHECK(r.sim_seconds == 115);
    CHECK(r.sim_nanos == 559291238);
    CHECK(r.value_count == 0);
    CHECK(r.checkpoint == N4S_CP_INTERMEDIATE);
}

/* "First CP Cleared" contains "CP Cleared" as a substring, so the order the
** classifier tests these strings in is load-bearing. */
static void test_checkpoint_classification(void)
{
    CHECK(n4s_classify_checkpoint("Track Cleared: [3]") == N4S_CP_TRACK);
    CHECK(n4s_classify_checkpoint("Lap Cleared: [2]") == N4S_CP_LAP);
    CHECK(n4s_classify_checkpoint("First CP Cleared: [0]") == N4S_CP_FIRST);
    CHECK(n4s_classify_checkpoint("CP Cleared: [1]") == N4S_CP_INTERMEDIATE);
    CHECK(n4s_classify_checkpoint("No further info") == N4S_CP_NONE);
    CHECK(n4s_classify_checkpoint(NULL) == N4S_CP_NONE);
}

static void test_trailing_newline(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response("1:OK:No errors so far:12.5:No further info\r\n",
        &r) == 0);
    CHECK_NEAR(r.values[0], 12.5, 1e-9);
    CHECK_STR(r.info, "No further info");
}

static void test_malformed(void)
{
    n4s_response_t r;

    CHECK(n4s_parse_response("", &r) == -1);
    CHECK(n4s_parse_response("not a response at all", &r) == -1);
    CHECK(n4s_parse_response(NULL, &r) == -1);
    /* An id but nothing after it: not enough to act on. */
    CHECK(n4s_parse_response("1", &r) == -1);
}

/* Lap counting must survive the news arriving on an unrelated response. */
static void test_lap_tracking(void)
{
    n4s_response_t r;

    n4s_track_reset();
    CHECK(n4s_track_laps() == 0);
    n4s_parse_response("1:OK:No errors so far:50.3:Lap Cleared: [4]", &r);
    n4s_track_observe(&r);
    n4s_parse_response("1:OK:No errors so far:50.3:No further info", &r);
    n4s_track_observe(&r);
    CHECK(n4s_track_laps() == 1);
    n4s_parse_response("1:OK:No errors so far:50.3:Lap Cleared: [8]", &r);
    n4s_track_observe(&r);
    CHECK(n4s_track_laps() == 2);
    CHECK(n4s_track_last_event() == N4S_CP_LAP);
    n4s_track_reset();
}

void test_response(void)
{
    printf("response\n");
    test_simple_scalar();
    test_error_response();
    test_no_payload_no_info();
    test_lidar_payload();
    test_info_containing_colons();
    test_simtime_payload();
    test_checkpoint_classification();
    test_trailing_newline();
    test_malformed();
    test_lap_tracking();
}
