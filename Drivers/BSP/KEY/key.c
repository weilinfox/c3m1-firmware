/**
 * @file        key.c
 * @brief       C3M1 non-blocking debounced button, film lever & BCD dial driver.
 */

#include "key.h"

#define KEY_DEBOUNCE_MS      25U
#define DIAL_DEBOUNCE_MS     35U
#define LONG_HOLD_MS         600U

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

static uint8_t  s_dial_stable   = 0;
static uint8_t  s_dial_last_raw = 0;
static uint32_t s_dial_tick     = 0;

static uint8_t read_bcd_raw(void)
{
    uint8_t val = 0;
    if (HAL_GPIO_ReadPin(TSR1_GPIO_Port, TSR1_Pin) == GPIO_PIN_SET) val |= 1U;
    if (HAL_GPIO_ReadPin(TSR2_GPIO_Port, TSR2_Pin) == GPIO_PIN_SET) val |= 2U;
    if (HAL_GPIO_ReadPin(TSR4_GPIO_Port, TSR4_Pin) == GPIO_PIN_SET) val |= 4U;
    if (HAL_GPIO_ReadPin(TSR8_GPIO_Port, TSR8_Pin) == GPIO_PIN_SET) val |= 8U;
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

    s_dial_stable = read_bcd_raw();
    s_dial_last_raw = s_dial_stable;
    s_dial_tick = now;
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

uint8_t key_read_shutter_dial(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw = read_bcd_raw();

    if (raw != s_dial_last_raw)
    {
        s_dial_last_raw = raw;
        s_dial_tick = now;
    }
    else if ((now - s_dial_tick) >= DIAL_DEBOUNCE_MS)
    {
        s_dial_stable = raw;
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
