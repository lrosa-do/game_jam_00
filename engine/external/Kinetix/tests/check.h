#ifndef KINETIX_TESTS_CHECK_H
#define KINETIX_TESTS_CHECK_H

#include <math.h>
#include <stdio.h>

static int gTestFailures = 0;

#define CHECK(condition)                                                                \
    do                                                                                  \
    {                                                                                   \
        if (!(condition))                                                               \
        {                                                                               \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
            ++gTestFailures;                                                            \
        }                                                                               \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                         \
    do                                                                                  \
    {                                                                                   \
        const double checkActual = (double)(actual);                                    \
        const double checkExpected = (double)(expected);                                \
        if (!(fabs(checkActual - checkExpected) <= (double)(tolerance)))                \
        {                                                                               \
            fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %g, expected %g +- %g\n",   \
                    __FILE__, __LINE__, #actual, checkActual, checkExpected,            \
                    (double)(tolerance));                                               \
            ++gTestFailures;                                                            \
        }                                                                               \
    } while (0)

#define TEST_RESULT()                                                                   \
    (gTestFailures == 0 ? (printf("OK\n"), 0)                                           \
                        : (fprintf(stderr, "%d check(s) failed\n", gTestFailures), 1))

#endif
