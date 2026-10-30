/*
** lidar-racer -- autonomous car AI driven by a 32-ray lidar
**
** tests/main.c -- runs every suite and reports.
**
** Exits non-zero on the first failure count above zero, so `make test` can be
** wired straight into CI or a container build without a wrapper script.
*/

#include <stdio.h>

#include "test.h"

int g_tests_run = 0;
int g_tests_failed = 0;

int main(void)
{
    test_util();
    test_car();
    test_response();

    printf("\n%d checks, %d failed\n", g_tests_run, g_tests_failed);
    if (g_tests_failed > 0) {
        printf("FAILED\n");
        return 1;
    }
    printf("OK\n");
    return 0;
}
