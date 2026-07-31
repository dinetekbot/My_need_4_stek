/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** link.c -- the transport: one command out, one response line back.
**
** Two rules govern this file, and breaking either one deadlocks the whole
** pipeline rather than producing a visible error:
**
** 1. Never buffer stdout. The AI sits in a loop of pipes
**    (ai -> buffer -> api -> buffer -> ai), so the API only answers once our
**    command actually reaches it. libc buffers stdout in 4K blocks when it is
**    a pipe, so a printf() sits in memory while we block reading the answer
**    to a command we never sent. We use write(2) exclusively.
**
** 2. Read exactly one line per command. The API answers every command,
**    including the ones it rejects, so falling out of step by one response
**    means every later decision is made on stale data.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "n4s.h"
#include "util.h"

/* Reused across calls so the driving loop performs no allocation per cycle. */
static char *g_line = NULL;
static size_t g_line_size = 0;

/* write(2) can return a short count on a pipe; loop until the whole command
** is out or the far end is gone. */
static int write_all(const char *data, size_t length)
{
    size_t done = 0;
    ssize_t written;

    while (done < length) {
        written = write(STDOUT_FILENO, data + done, length - done);
        if (written <= 0)
            return -1;
        done += (size_t)written;
    }
    return 0;
}

static int send_command(const char *command)
{
    char buffer[N4S_LINE_MAX];
    int length;

    length = snprintf(buffer, sizeof(buffer), "%s\n", command);
    if (length <= 0 || (size_t)length >= sizeof(buffer))
        return -1;
    return write_all(buffer, (size_t)length);
}

int n4s_request(const char *command, n4s_response_t *out)
{
    n4s_response_t local;
    n4s_response_t *target = (out != NULL) ? out : &local;

    if (command == NULL)
        return -1;
    if (send_command(command) != 0)
        return -1;
    if (getline(&g_line, &g_line_size, stdin) <= 0)
        return -1;
    if (n4s_parse_response(g_line, target) != 0)
        return -1;
    /* Checkpoint news rides along on whatever response happens to be next,
    ** so every single one is inspected here rather than at the call sites. */
    n4s_track_observe(target);
    return 0;
}

int n4s_request_f(const char *command, double argument, n4s_response_t *out)
{
    char buffer[N4S_LINE_MAX];
    int length;

    length = snprintf(buffer, sizeof(buffer), "%s:%.3f", command, argument);
    if (length <= 0 || (size_t)length >= sizeof(buffer))
        return -1;
    return n4s_request(buffer, out);
}
