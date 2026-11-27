/**
 ****************************************************************************************************
 * @file        stmflash.h
 * @brief       STM32F103 internal Flash read helpers & wear-leveled shutter counter page driver.
 *
 * Page layout at page_addr (2 KB = 1024 halfwords, STM32F103ZET6):
 *   - STOCK mode (factory compatible):
 *       slot[0]         = uint16_t shutter count (0..65534; 0xFFFF = blank -> 0)
 *       slot[1..1023]   = 0xFFFF
 *   - WEAR-LVL mode (1022-slot append-only log, >10M actuations):
 *       slot[0..1021]   = chronological uint16_t counter log (0..65534; 0xFFFF = erased/unused)
 *       slot[1022]      = STMFLASH_CNT_MAGIC_LO (0x574C, "WL")
 *       slot[1023]      = STMFLASH_CNT_MAGIC_HI (0x4333, "C3")
 *
 * Auto-detection:
 *   Reads slot[1022..1023]; presence of "C3WL" selects WEAR-LVL, otherwise STOCK.
 *   Requires 0 B static SRAM buffer (g_flashbuf[1024] removed -> saves 2048 B .bss).
 ****************************************************************************************************
 */

#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "../../SYSTEM/sys/sys.h"

/* STM32F103ZET6 internal Flash geometry */
#define STM32_FLASH_SIZE        0x80000UL       /* 512 KB total Flash */
#define STM32_FLASH_BASE        0x08000000UL    /* Flash base address */

#if STM32_FLASH_SIZE < 256 * 1024
#define STM32_SECTOR_SIZE       1024UL          /* 1 KB page */
#else
#define STM32_SECTOR_SIZE       2048UL          /* 2 KB page (high-density F103) */
#endif

/* Shutter counter storage format modes */
#define STMFLASH_CNT_MODE_STOCK 0U              /* Legacy single-slot at page+0 (factory compatible) */
#define STMFLASH_CNT_MODE_WL    1U              /* 1022-slot append-only wear leveling */

#define STMFLASH_CNT_HALF_PER_PAGE  (STM32_SECTOR_SIZE / 2UL)
#define STMFLASH_CNT_SLOTS          (STMFLASH_CNT_HALF_PER_PAGE - 2UL) /* 1022 slots */
#define STMFLASH_CNT_MAGIC_LO       0x574CU     /* slot[1022] */
#define STMFLASH_CNT_MAGIC_HI       0x4333U     /* slot[1023] */
#define STMFLASH_CNT_MAX_VAL        65534U      /* 0xFFFE: reserve 0xFFFF strictly as erased sentinel */

/* Low-level read-only Flash accessors */
uint16_t stmflash_read_halfword(uint32_t faddr);
void     stmflash_read(uint32_t raddr, uint16_t *pbuf, uint16_t length);

/* Wear-leveled / stock-compatible counter page API (0 B .bss, HAL error + readback checked) */
uint8_t  stmflash_counter_detect_mode(uint32_t page_addr);
uint8_t  stmflash_counter_load(uint32_t page_addr, uint16_t *out_val,
                               uint8_t *out_mode, uint16_t *out_slots_used);
uint8_t  stmflash_counter_save(uint32_t page_addr, uint16_t val);
uint8_t  stmflash_counter_set_mode(uint32_t page_addr, uint8_t target_mode, uint16_t cur_val);

#endif /* __STMFLASH_H */
