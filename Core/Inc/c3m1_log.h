/**
 * @file        c3m1_log.h
 * @brief       Compile-time UART debug logging gate (default OFF in production; build with `make DEBUG_LOG=1`).
 */

#ifndef C3M1_LOG_H_
#define C3M1_LOG_H_

#ifndef C3M1_DEBUG_LOG
#define C3M1_DEBUG_LOG 0
#endif

#if C3M1_DEBUG_LOG
#include <stdio.h>
#define C3M1_LOG(...) printf(__VA_ARGS__)
#else
#define C3M1_LOG(...) ((void)0)
#endif

#endif /* C3M1_LOG_H_ */
