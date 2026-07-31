/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** test.h -- a test harness in thirty lines.
**
** No framework, no dependency to install, no version to pin. `make test`
** works on a bare toolchain, which is the point: a test suite that needs
** setting up is a test suite nobody runs.
*/

#ifndef TEST_H
#define TEST_H

#include <math.h>
#include <stdio.h>
#include <string.h>

extern int g_tests_run;
extern int g_tests_failed;

#define CHECK(condition)                                                     \
    do {                                                                     \
        g_tests_run++;                                                       \
        if (!(condition)) {                                                  \
            g_tests_failed++;                                                \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition);    \
        }                                                                    \
    } while (0)

/* Floating point equality with a tolerance, because 0.1 + 0.2 != 0.3. */
#define CHECK_NEAR(actual, expected, tolerance)                              \
    do {                                                                     \
        double a_ = (actual);                                                \
        double e_ = (expected);                                              \
        g_tests_run++;                                                       \
        if (fabs(a_ - e_) > (tolerance)) {                                   \
            g_tests_failed++;                                                \
            printf("  FAIL %s:%d  %s: got %g, expected %g (+-%g)\n",         \
                __FILE__, __LINE__, #actual, a_, e_, (double)(tolerance));   \
        }                                                                    \
    } while (0)

#define CHECK_STR(actual, expected)                                          \
    do {                                                                     \
        const char *a_ = (actual);                                           \
        const char *e_ = (expected);                                         \
        g_tests_run++;                                                       \
        if (a_ == NULL || strcmp(a_, e_) != 0) {                             \
            g_tests_failed++;                                                \
            printf("  FAIL %s:%d  %s: got \"%s\", expected \"%s\"\n",        \
                __FILE__, __LINE__, #actual, a_ ? a_ : "(null)", e_);        \
        }                                                                    \
    } while (0)

void test_response(void);
void test_perception(void);
void test_control(void);
void test_util(void);

#endif /* TEST_H */
