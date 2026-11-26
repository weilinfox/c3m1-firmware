/*---------------------------------------------------------------------------/
/  Configurations of FatFs Module  R0.15a  (C3M1 Firmware: FAT12/16/32 + exFAT)
/---------------------------------------------------------------------------*/

#define FFCONF_DEF	5380	/* Revision ID (matches FatFs R0.15a ff.h) */

/*---------------------------------------------------------------------------/
/ Additional user headers required by STM32 FatFs diskio glue
/---------------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "bsp_driver_sd.h"

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_READONLY	0
#define FF_FS_MINIMIZE	0
#define FF_USE_FIND		0
#define FF_USE_MKFS		1
#define FF_USE_FASTSEEK	0
#define FF_USE_EXPAND	0
#define FF_USE_CHMOD	0
#define FF_USE_LABEL	0
#define FF_USE_FORWARD	0

#define FF_USE_STRFUNC	0
#define FF_PRINT_LLI	0
#define FF_PRINT_FLOAT	0
#define FF_STRF_ENCODE	3

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define FF_CODE_PAGE	437	/* U.S. OEM (compact ~0.5 KB table; ASCII paths CAYE_CAM/CDIxxxxx.dng) */

#define FF_USE_LFN		1	/* 1: Static BSS working buffer (required for exFAT) */
#define FF_MAX_LFN		255

#define FF_LFN_UNICODE	0	/* 0: ANSI/OEM (TCHAR = char) */
#define FF_LFN_BUF		255
#define FF_SFN_BUF		12

#define FF_FS_RPATH		0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define FF_VOLUMES		1
#define FF_STR_VOLUME_ID	0
#define FF_VOLUME_STRS		"SD"
#define FF_MULTI_PARTITION	0

#define FF_MIN_SS		512
#define FF_MAX_SS		512

#define FF_LBA64		0	/* 32-bit LBA = 2^32 * 512 B = 2 TB max (full SDXC ceiling) */
#define FF_MIN_GPT		0x10000000
#define FF_USE_TRIM		0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_TINY		0
#define FF_FS_EXFAT		1	/* Enable exFAT for SDXC cards (64 GB ~ 2 TB) */

#define FF_FS_NORTC		0	/* Use get_fattime() backed by c3m1_clock */
/* Legacy hardcoded date kept in source for reference only; unused at runtime when FF_FS_NORTC == 0 */
#define FF_NORTC_MON	12
#define FF_NORTC_MDAY	26
#define FF_NORTC_YEAR	2026

#define FF_FS_NOFSINFO	0
#define FF_FS_LOCK		5
#define FF_FS_REENTRANT	0
#define FF_FS_TIMEOUT	1000

/*--- End of configuration options ---*/
