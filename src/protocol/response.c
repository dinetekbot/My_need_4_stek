/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** response.c -- parsing a response line.
**
** The format looks trivial:
**
**     VALUE_ID:STATUS:CODE_STR[:payload...]:ADDITIONAL_INFO
**
** but it has one trap that is easy to miss and expensive to get wrong:
** ADDITIONAL_INFO is free text and contains colons of its own. A real line
** off the wire looks like
**
**     1:OK:No errors so far:-25.1:First CP Cleared: [0][88s 26029102ns]
**
** Splitting that on ':' and reading fixed indices gives you "First CP Cleared"
** in one field and "[0][88s 26029102ns]" in another, so any code looking for
** the exact string "First CP Cleared: ..." finds nothing and the AI never
** learns that it crossed a checkpoint.
**
** So we parse positionally for the three mandatory fields, then greedily
** consume numeric fields, and treat everything left as one opaque string.
*/

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "n4s.h"
#include "util.h"

/*
** Copy the text between `*pos` and the next ':' (or end of string) into
** `out`. Advances `*pos` past the separator. Returns false when there is
** nothing left to read.
*/
static bool next_field(const char *line, size_t *pos, char *out,
    size_t out_size)
{
    size_t length = 0;

    if (line[*pos] == '\0')
        return false;
    while (line[*pos] != '\0' && line[*pos] != ':') {
        if (length + 1 < out_size)
            out[length++] = line[*pos];
        (*pos)++;
    }
    out[length] = '\0';
    if (line[*pos] == ':')
        (*pos)++;
    return true;
}

/* Case-insensitive substring search; strcasestr is not portable enough. */
static bool contains_ci(const char *haystack, const char *needle)
{
    size_t nlen = strlen(needle);

    if (nlen == 0)
        return true;
    for (; *haystack != '\0'; haystack++) {
        size_t i = 0;

        while (i < nlen && haystack[i] != '\0'
            && tolower((unsigned char)haystack[i])
            == tolower((unsigned char)needle[i]))
            i++;
        if (i == nlen)
            return true;
    }
    return false;
}

n4s_checkpoint_t n4s_classify_checkpoint(const char *info)
{
    if (info == NULL)
        return N4S_CP_NONE;
    /* Order matters: "First CP Cleared" also contains "CP Cleared", and
    ** "Track Cleared" is the one we must never miss. */
    if (contains_ci(info, "Track Cleared"))
        return N4S_CP_TRACK;
    if (contains_ci(info, "Lap Cleared"))
        return N4S_CP_LAP;
    if (contains_ci(info, "First CP Cleared"))
        return N4S_CP_FIRST;
    if (contains_ci(info, "CP Cleared"))
        return N4S_CP_INTERMEDIATE;
    return N4S_CP_NONE;
}

/* Recognise a simulation timestamp field, e.g. "[115s,559291238ns]". */
static bool parse_simtime(const char *field, long *seconds, long *nanos)
{
    return sscanf(field, "[%lds,%ldns]", seconds, nanos) == 2;
}

/* Strip the trailing newline, carriage return and spaces in place. */
static void rstrip(char *text)
{
    size_t length = strlen(text);

    while (length > 0 && (text[length - 1] == '\n' || text[length - 1] == '\r'
        || text[length - 1] == ' '))
        text[--length] = '\0';
}

/*
** Read the mandatory VALUE_ID:STATUS:CODE_STR prefix.
** Returns the offset where the payload starts, or 0 on failure.
*/
static size_t parse_header(const char *line, n4s_response_t *out)
{
    char field[N4S_CODE_MAX];
    size_t pos = 0;
    double id_value;

    if (!next_field(line, &pos, field, sizeof(field)))
        return 0;
    if (!parse_double(field, &id_value))
        return 0;
    out->value_id = (int)id_value;
    if (!next_field(line, &pos, field, sizeof(field)))
        return 0;
    out->ok = (strcmp(field, "OK") == 0);
    if (!next_field(line, &pos, out->code, sizeof(out->code)))
        return 0;
    return pos;
}

/*
** Consume the numeric payload, then keep the rest of the line verbatim.
** A field that is not a number ends the payload -- that is how we find the
** boundary between the data and the free text without counting colons.
*/
static void parse_payload(const char *line, size_t pos, n4s_response_t *out)
{
    char field[N4S_INFO_MAX];
    size_t mark = pos;
    double value;

    while (next_field(line, &pos, field, sizeof(field))) {
        if (parse_double(field, &value)) {
            if (out->value_count < N4S_LIDAR_RAYS)
                out->values[out->value_count++] = value;
            mark = pos;
            continue;
        }
        if (!out->has_simtime
            && parse_simtime(field, &out->sim_seconds, &out->sim_nanos)) {
            out->has_simtime = true;
            mark = pos;
            continue;
        }
        break;
    }
    /* `mark` is the first byte that is not part of the payload. Everything
    ** from there on is ADDITIONAL_INFO, colons included. */
    strncpy(out->info, line + mark, sizeof(out->info) - 1);
    out->info[sizeof(out->info) - 1] = '\0';
    rstrip(out->info);
}

int n4s_parse_response(const char *line, n4s_response_t *out)
{
    char buffer[N4S_LINE_MAX];
    size_t pos;

    if (line == NULL || out == NULL)
        return -1;
    memset(out, 0, sizeof(*out));
    strncpy(buffer, line, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    rstrip(buffer);
    pos = parse_header(buffer, out);
    if (pos == 0)
        return -1;
    parse_payload(buffer, pos, out);
    out->checkpoint = n4s_classify_checkpoint(out->info);
    return 0;
}
