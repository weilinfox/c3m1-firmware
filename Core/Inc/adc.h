/**
 * @file    adc.h
 * @brief   C3M1 battery voltage ADC1 driver (PA1 / ADC1_IN1, 10k/10k divider).
 */

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void     MX_ADC1_Init(void);
uint16_t adc_battery_read_raw(void);
uint16_t adc_battery_get_mv(void);
uint8_t  adc_battery_get_percent(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
