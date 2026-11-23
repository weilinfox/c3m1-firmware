/**
 * @file        key.c
 * @brief       C3M1 non-blocking debounced button, film lever & BCD dial driver.
 */

#include "key.h"

#define KEY_DEBOUNCE_MS      25U
#define DIAL_DEBOUNCE_MS     35U
#define NUMRST_HOLD_MS       1500U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       stable_pressed;   /* 1 = pressed (active LOW) */
    uint8_t       last_raw;
    uint32_t      change_tick;
    uint8_t       evt_bit;
} btn_state_t;

static btn_state_t s_btns[4] = {
    { KEYISO_GPIO_Port,  KEYISO_Pin,  0, 0, 0, BTN_EVT_ISO  },
    { KEYSHOT_GPIO_Port, KEYSHOT_Pin, 0, 0, 0, BTN_EVT_SHOT },
    { KEY4X_GPIO_Port,   KEY4X_Pin,   0, 0, 0, BTN_EVT_4X   },
    { FILMKEY_GPIO_Port, FILMKEY_Pin, 0, 0, 0, BTN_EVT_FILM },
};

static uint8_t  s_numrst_active = 0;
static uint8_t  s_numrst_fired  = 0;
static uint32_t s_numrst_tick   = 0;

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
    }

    s_dial_stable = read_bcd_raw();
    s_dial_last_raw = s_dial_stable;
    s_dial_tick = now;
}

uint8_t key_poll_events(void)
{
    uint8_t events = BTN_EVT_NONE;
    uint32_t now = HAL_GetTick();

    /* Poll 4 debounced active-LOW buttons / switches */
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

    /* Poll NUMRST (PA3) long press >= 1.5s */
    uint8_t numrst_raw = (HAL_GPIO_ReadPin(NUMRST_GPIO_Port, NUMRST_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    if (numrst_raw)
    {
        if (!s_numrst_active)
        {
            s_numrst_active = 1;
            s_numrst_fired = 0;
            s_numrst_tick = now;
        }
        else if (!s_numrst_fired && (now - s_numrst_tick) >= NUMRST_HOLD_MS)
        {
            s_numrst_fired = 1;
            events |= BTN_EVT_NUMRST_LONG;
        }
    }
    else
    {
        s_numrst_active = 0;
        s_numrst_fired = 0;
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
