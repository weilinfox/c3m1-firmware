/**
 ****************************************************************************************************
 * @file        stmflash.c
 * @brief       STM32F103 internal Flash read helpers & wear-leveled shutter counter page driver.
 ****************************************************************************************************
 */

#include "stmflash.h"

uint16_t stmflash_read_halfword(uint32_t faddr)
{
    return *(volatile uint16_t *)faddr;
}

void stmflash_read(uint32_t raddr, uint16_t *pbuf, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++)
    {
        pbuf[i] = stmflash_read_halfword(raddr);
        raddr += 2UL;
    }
}

static uint8_t flash_page_valid(uint32_t page_addr)
{
    if (page_addr < STM32_FLASH_BASE ||
        page_addr > (STM32_FLASH_BASE + STM32_FLASH_SIZE - STM32_SECTOR_SIZE))
    {
        return 0U;
    }
    if ((page_addr % STM32_SECTOR_SIZE) != 0UL)
    {
        return 0U;
    }
    return 1U;
}

static uint16_t flash_sanitize_val(uint16_t val)
{
    /* Reserve 0xFFFF strictly as the erased-slot sentinel */
    return (val == 0xFFFFU) ? STMFLASH_CNT_MAX_VAL : val;
}

static uint8_t flash_tail_is_erased(uint32_t page_addr, uint16_t start_slot, uint16_t end_slot_excl)
{
    for (uint16_t i = start_slot; i < end_slot_excl; i++)
    {
        if (stmflash_read_halfword(page_addr + (uint32_t)i * 2UL) != 0xFFFFU)
        {
            return 0U;
        }
    }
    return 1U;
}

/* Caller must hold HAL_FLASH_Unlock() */
static uint8_t flash_erase_page(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase_op;
    uint32_t page_err = 0UL;

    erase_op.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_op.Banks       = FLASH_BANK_1;
    erase_op.PageAddress = page_addr;
    erase_op.NbPages     = 1U;

    if (HAL_FLASHEx_Erase(&erase_op, &page_err) != HAL_OK)
    {
        return 0U;
    }
    if (page_err != 0xFFFFFFFFUL)
    {
        return 0U;
    }
    return 1U;
}

/* Caller must hold HAL_FLASH_Unlock(); verifies programmed halfword immediately */
static uint8_t flash_program_hw(uint32_t addr, uint16_t val)
{
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, val) != HAL_OK)
    {
        return 0U;
    }
    if (stmflash_read_halfword(addr) != val)
    {
        return 0U;
    }
    return 1U;
}

/* Caller must hold HAL_FLASH_Unlock().
 * Power-loss-safe order: erase -> write slot[0] FIRST -> write magic LAST.
 * If power fails after slot[0] but before magic completes, next boot reads slot[0]
 * in STOCK mode with zero count loss. */
static uint8_t flash_rewrite_page(uint32_t page_addr, uint16_t val, uint8_t mode)
{
    if (!flash_erase_page(page_addr))
    {
        return 0U;
    }
    if (!flash_program_hw(page_addr, val))
    {
        return 0U;
    }
    if (mode == STMFLASH_CNT_MODE_WL)
    {
        uint32_t magic_lo_addr = page_addr + (uint32_t)STMFLASH_CNT_SLOTS * 2UL;
        uint32_t magic_hi_addr = magic_lo_addr + 2UL;
        if (!flash_program_hw(magic_lo_addr, STMFLASH_CNT_MAGIC_LO))
        {
            return 0U;
        }
        if (!flash_program_hw(magic_hi_addr, STMFLASH_CNT_MAGIC_HI))
        {
            return 0U;
        }
    }
    return 1U;
}

/* Scan WL slots backward so next_slot = last_used + 1 is guaranteed erased (or full).
 * Returns count of used slots (0..STMFLASH_CNT_SLOTS) and writes latest value to *out_val. */
static uint16_t flash_wl_scan(uint32_t page_addr, uint16_t *out_val)
{
    for (int32_t i = (int32_t)STMFLASH_CNT_SLOTS - 1L; i >= 0L; i--)
    {
        uint16_t v = stmflash_read_halfword(page_addr + (uint32_t)i * 2UL);
        if (v != 0xFFFFU)
        {
            *out_val = v;
            return (uint16_t)(i + 1L);
        }
    }
    *out_val = 0U;
    return 0U;
}

uint8_t stmflash_counter_detect_mode(uint32_t page_addr)
{
    if (!flash_page_valid(page_addr))
    {
        return STMFLASH_CNT_MODE_STOCK;
    }
    uint32_t magic_lo_addr = page_addr + (uint32_t)STMFLASH_CNT_SLOTS * 2UL;
    uint16_t lo = stmflash_read_halfword(magic_lo_addr);
    uint16_t hi = stmflash_read_halfword(magic_lo_addr + 2UL);
    if (lo == STMFLASH_CNT_MAGIC_LO && hi == STMFLASH_CNT_MAGIC_HI)
    {
        return STMFLASH_CNT_MODE_WL;
    }
    return STMFLASH_CNT_MODE_STOCK;
}

uint8_t stmflash_counter_load(uint32_t page_addr, uint16_t *out_val,
                              uint8_t *out_mode, uint16_t *out_slots_used)
{
    if (!flash_page_valid(page_addr) || out_val == NULL)
    {
        return 0U;
    }

    uint8_t mode = stmflash_counter_detect_mode(page_addr);
    uint16_t val = 0U;
    uint16_t used = 0U;

    if (mode == STMFLASH_CNT_MODE_WL)
    {
        used = flash_wl_scan(page_addr, &val);
    }
    else
    {
        uint16_t v0 = stmflash_read_halfword(page_addr);
        if (v0 != 0xFFFFU)
        {
            val  = v0;
            used = 1U;
        }
    }

    *out_val = val;
    if (out_mode != NULL)       *out_mode       = mode;
    if (out_slots_used != NULL) *out_slots_used = used;
    return 1U;
}

uint8_t stmflash_counter_save(uint32_t page_addr, uint16_t val)
{
    if (!flash_page_valid(page_addr))
    {
        return 0U;
    }
    val = flash_sanitize_val(val);

    uint8_t mode = stmflash_counter_detect_mode(page_addr);
    uint8_t ok   = 0U;

    if (mode == STMFLASH_CNT_MODE_WL)
    {
        uint16_t cur_val = 0U;
        uint16_t next_slot = flash_wl_scan(page_addr, &cur_val);
        if (next_slot > 0U && cur_val == val)
        {
            return 1U; /* Already persisted at latest slot */
        }

        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            return 0U;
        }

        if (next_slot < STMFLASH_CNT_SLOTS)
        {
            ok = flash_program_hw(page_addr + (uint32_t)next_slot * 2UL, val);
        }
        else
        {
            /* All 1022 slots full -> single page erase + wrap back to slot[0] */
            ok = flash_rewrite_page(page_addr, val, STMFLASH_CNT_MODE_WL);
        }

        (void)HAL_FLASH_Lock();
        return ok;
    }
    else
    {
        uint16_t v0 = stmflash_read_halfword(page_addr);
        uint8_t  tail_clean = flash_tail_is_erased(page_addr, 1U, (uint16_t)STMFLASH_CNT_HALF_PER_PAGE);
        if (v0 == val && tail_clean)
        {
            return 1U; /* Exact factory page already present */
        }

        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            return 0U;
        }

        if (v0 == 0xFFFFU && tail_clean)
        {
            /* Blank page: program slot[0] directly with zero erase */
            ok = flash_program_hw(page_addr, val);
        }
        else
        {
            ok = flash_rewrite_page(page_addr, val, STMFLASH_CNT_MODE_STOCK);
        }

        (void)HAL_FLASH_Lock();
        return ok;
    }
}

uint8_t stmflash_counter_set_mode(uint32_t page_addr, uint8_t target_mode, uint16_t cur_val)
{
    if (!flash_page_valid(page_addr) || target_mode > STMFLASH_CNT_MODE_WL)
    {
        return 0U;
    }
    cur_val = flash_sanitize_val(cur_val);

    uint8_t cur_mode = stmflash_counter_detect_mode(page_addr);
    if (cur_mode == target_mode)
    {
        return 1U;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 0U;
    }

    uint8_t ok = 0U;
    if (target_mode == STMFLASH_CNT_MODE_WL)
    {
        /* Fast zero-erase in-place upgrade when converting from a clean STOCK page:
         * slot[0] already holds cur_val (or 0xFFFF) and slot[1..1023] are all 0xFFFF. */
        uint16_t v0 = stmflash_read_halfword(page_addr);
        uint8_t  tail_clean = flash_tail_is_erased(page_addr, 1U, (uint16_t)STMFLASH_CNT_HALF_PER_PAGE);
        if (tail_clean && (v0 == cur_val || v0 == 0xFFFFU))
        {
            ok = 1U;
            if (v0 == 0xFFFFU)
            {
                ok = flash_program_hw(page_addr, cur_val);
            }
            if (ok)
            {
                uint32_t magic_lo_addr = page_addr + (uint32_t)STMFLASH_CNT_SLOTS * 2UL;
                ok = flash_program_hw(magic_lo_addr, STMFLASH_CNT_MAGIC_LO);
                if (ok)
                {
                    ok = flash_program_hw(magic_lo_addr + 2UL, STMFLASH_CNT_MAGIC_HI);
                }
            }
        }
        else
        {
            ok = flash_rewrite_page(page_addr, cur_val, STMFLASH_CNT_MODE_WL);
        }
    }
    else
    {
        /* Convert WL -> STOCK: single erase + write cur_val at slot[0] only (factory compatible) */
        ok = flash_rewrite_page(page_addr, cur_val, STMFLASH_CNT_MODE_STOCK);
    }

    (void)HAL_FLASH_Lock();
    return ok;
}
