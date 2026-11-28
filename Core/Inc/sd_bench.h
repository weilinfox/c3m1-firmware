#ifndef C3M1_SD_BENCH_H
#define C3M1_SD_BENCH_H

#include <stdint.h>
#include "ff.h"

#ifndef C3M1_SD_BENCH
#define C3M1_SD_BENCH 0
#endif

#if C3M1_SD_BENCH
void sd_bench_run_ui(FATFS *fs_ptr, FIL *fil_ptr, uint8_t *io_buf_4k);
#endif

#endif /* C3M1_SD_BENCH_H */
