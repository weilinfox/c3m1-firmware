/**
 * @file        key.c
 * @brief       C3M1 non-blocking debounced button, film lever & BCD dial driver.
 */

#include "key.h"

#define KEY_DEBOUNCE_MS       25U
#define DIAL_POLL_INTERVAL_MS 8U        /* Throttle BCD scans to <= 125 Hz (matches 10 ms menu/pb loops) */
#define DIAL_DEBOUNCE_MS      55U
#define DIAL_MIN_SAMPLES      3U        /* Require >= 3 consecutive matching polls */
#define DIAL_STALE_GAP_MS     250U      /* Reset debounce after long blocking SD/LCD ops */
#define LONG_HOLD_MS          600U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       stable_pressed;   /* 1 = pressed (active LOW) */
    uint8_t       last_raw;
    uint32_t      change_tick;
    uint8_t       evt_bit;
    uint8_t       long_evt_bit;     /* Non-zero: short fires on release, long fires at LONG_HOLD_MS */
    uint32_t      press_tick;
    uint8_t       long_fired;
} btn_state_t;

/* 0: KEYISO (short/long), 1: KEYSHOT (immediate), 2: KEY4X (short/long), 3: FILMKEY (short/long) */
static btn_state_t s_btns[4] = {
    { KEYISO_GPIO_Port,  KEYISO_Pin,  0, 0, 0, BTN_EVT_ISO,  BTN_EVT_ISO_LONG,  0, 0 },
    { KEYSHOT_GPIO_Port, KEYSHOT_Pin, 0, 0, 0, BTN_EVT_SHOT, BTN_EVT_NONE,      0, 0 },
    { KEY4X_GPIO_Port,   KEY4X_Pin,   0, 0, 0, BTN_EVT_4X,   BTN_EVT_4X_LONG,   0, 0 },
    { FILMKEY_GPIO_Port, FILMKEY_Pin, 0, 0, 0, BTN_EVT_FILM, BTN_EVT_FILM_LONG, 0, 0 },
};

static uint8_t  s_dial_stable    = 0;
static uint8_t  s_dial_last_raw  = 0;
static uint32_t s_dial_tick      = 0;
static uint32_t s_dial_last_poll = 0;
static uint8_t  s_dial_same_cnt  = 0;

static void bcd_settle_delay(void)
{
    /* ~25 us @ 72 MHz: >10x RC time constant of 40k pull-down / 10k pull-up on FPC traces */
    for (volatile uint32_t i = 0U; i < 300U; i++)
    {
        __NOP();
    }
}

static uint8_t read_bcd_raw(void)
{
    /*
     * Board2 SW6 (TSR-16 LG) ties common C to VCC3.3 through R10=10k while PF6..PF9
     * use STM32 internal ~40k pull-downs. Sampling all 4 pins simultaneously drags
     * 3/4-bit-HIGH positions (7, 11, 13, 14, 15/AUTO) down to ~1.4-1.88V near V_IH,
     * causing random bit drops and +/-2 or +/-8 phantom dial jumps.
     * Sampling one pin with pull-down at a time (other 3 high-Z floating) loads R10
     * with only a single 40k resistor -> solid ~2.5-2.64V HIGH across all 16 detents.
     */
    uint8_t val = 0U;

    /* Ensure ODR bits 6..9 are 0 so CNF=10 selects input pull-down (atomic BRR) */
    GPIOF->BRR = (TSR1_Pin | TSR2_Pin | TSR4_Pin | TSR8_Pin);

    uint32_t crl_float = (GPIOF->CRL & 0x00FFFFFFUL) | 0x44000000UL;
    uint32_t crh_float = (GPIOF->CRH & 0xFFFFFF00UL) | 0x00000044UL;

    /* Sample PF6 (TSR1, bit 0) */
    GPIOF->CRH = crh_float;
    GPIOF->CRL = (crl_float & 0xF0FFFFFFUL) | 0x08000000UL;
    bcd_settle_delay();
    if ((GPIOF->IDR & TSR1_Pin) != 0U) val |= 1U;

    /* Sample PF7 (TSR2, bit 1) */
    GPIOF->CRL = (crl_float & 0x0FFFFFFFUL) | 0x80000000UL;
    bcd_settle_delay();
    if ((GPIOF->IDR & TSR2_Pin) != 0U) val |= 2U;

    /* Sample PF8 (TSR4, bit 2) */
    GPIOF->CRL = crl_float;
    GPIOF->CRH = (crh_float & 0xFFFFFFF0UL) | 0x00000008UL;
    bcd_settle_delay();
    if ((GPIOF->IDR & TSR4_Pin) != 0U) val |= 4U;

    /* Sample PF9 (TSR8, bit 3) */
    GPIOF->CRH = (crh_float & 0xFFFFFF0FUL) | 0x00000080UL;
    bcd_settle_delay();
    if ((GPIOF->IDR & TSR8_Pin) != 0U) val |= 8U;

    /* Restore all 4 pins to input pull-down when idle (avoids floating CMOS inputs) */
    GPIOF->CRL = (crl_float & 0x00FFFFFFUL) | 0x88000000UL;
    GPIOF->CRH = (crh_float & 0xFFFFFF00UL) | 0x00000088UL;

    return val;
}

void key_init_state(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t pressed = (HAL_GPIO_ReadPin(s_btns[i].port, s_btns[i].pin) == GPIO_PIN_RESET) ? 1U : 0U;
        s_btns[i].stable_pressed = pressed;
        s_btns[i].last_raw = pressed;
        s_btns[i].change_tick = now;
        s_btns[i].press_tick = now;
        s_btns[i].long_fired = pressed;
    }

    s_dial_stable    = read_bcd_raw();
    s_dial_last_raw  = s_dial_stable;
    s_dial_tick      = now;
    s_dial_last_poll = now;
    s_dial_same_cnt  = DIAL_MIN_SAMPLES;
}

uint8_t key_poll_events(void)
{
    uint8_t events = BTN_EVT_NONE;
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(s_btns[i].port, s_btns[i].pin) == GPIO_PIN_RESET) ? 1U : 0U;
        if (raw != s_btns[i].last_raw)
        {
            s_btns[i].last_raw = raw;
            s_btns[i].change_tick = now;
        }
        else if ((now - s_btns[i].change_tick) >= KEY_DEBOUNCE_MS)
        {
            if (s_btns[i].long_evt_bit != BTN_EVT_NONE)
            {
                /* Dual-action button: short press on release, long press at >= LONG_HOLD_MS */
                if (raw != s_btns[i].stable_pressed)
                {
                    s_btns[i].stable_pressed = raw;
                    if (raw == 1U)
                    {
                        s_btns[i].press_tick = now;
                        s_btns[i].long_fired = 0U;
                    }
                    else if (!s_btns[i].long_fired)
                    {
                        events |= s_btns[i].evt_bit;
                    }
                }
                else if (raw == 1U && !s_btns[i].long_fired && (now - s_btns[i].press_tick) >= LONG_HOLD_MS)
                {
                    s_btns[i].long_fired = 1U;
                    events |= s_btns[i].long_evt_bit;
                }
            }
            else
            {
                /* Immediate-action button (KEYSHOT / KEY4X): fire on press-down */
                if (raw != s_btns[i].stable_pressed)
                {
                    s_btns[i].stable_pressed = raw;
                    if (raw == 1U)
                    {
                        events |= s_btns[i].evt_bit;
                    }
                }
            }
        }
    }

    return events;
}

void key_dial_reset_debounce(void)
{
    uint32_t now = HAL_GetTick();
    s_dial_last_raw  = s_dial_stable;
    s_dial_tick      = now;
    s_dial_last_poll = now;
    s_dial_same_cnt  = 0U;
}

uint8_t key_read_shutter_dial(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t gap = now - s_dial_last_poll;
    if (gap < DIAL_POLL_INTERVAL_MS)
    {
        return s_dial_stable;
    }
    s_dial_last_poll = now;

    uint8_t raw = read_bcd_raw();
    if (raw != s_dial_last_raw || gap > DIAL_STALE_GAP_MS)
    {
        s_dial_last_raw = raw;
        s_dial_tick     = now;
        s_dial_same_cnt = 1U;
    }
    else
    {
        if (s_dial_same_cnt < 255U) s_dial_same_cnt++;
        if (s_dial_same_cnt >= DIAL_MIN_SAMPLES && (now - s_dial_tick) >= DIAL_DEBOUNCE_MS)
        {
            s_dial_stable = raw;
        }
    }

    return s_dial_stable;
}

uint8_t key_any_top_button_pressed(void)
{
    if (HAL_GPIO_ReadPin(KEYISO_GPIO_Port,  KEYISO_Pin)  == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(KEYSHOT_GPIO_Port, KEYSHOT_Pin) == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(KEY4X_GPIO_Port,   KEY4X_Pin)   == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(FILMKEY_GPIO_Port, FILMKEY_Pin) == GPIO_PIN_RESET) return 1U;
    return 0U;
}

uint8_t key_is_shot_pressed(void)
{
    return (HAL_GPIO_ReadPin(KEYSHOT_GPIO_Port, KEYSHOT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}
