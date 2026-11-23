/**
 * @file    adc.c
 * @brief   C3M1 battery voltage ADC1 driver using CMSIS registers (PA1 / ADC1_IN1).
 *          Hardware: Vbat -> R17(10k) -> PA1 (C16=100nF) -> R18(10k) -> GND.
 */

#include "adc.h"
#include "../../SYSTEM/delay/delay.h"

#define ADC_TIMEOUT_CYCLES  50000U
#define ADC_VREF_MV         3300UL
#define BAT_DIVIDER_RATIO   2UL     /* 10k / 10k voltage divider */
#define BAT_EMPTY_MV        3350U
#define BAT_FULL_MV         4180U

void MX_ADC1_Init(void)
{
    uint32_t timeout;

    /* ADCCLK = PCLK2 / 6 = 72MHz / 6 = 12MHz (STM32F103 max is 14MHz) */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Reset ADC1 control registers */
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    /* Channel 1 (PA1) sample time = 239.5 cycles for stable reading over 10k divider */
    ADC1->SMPR2 &= ~(0x7UL << 3);
    ADC1->SMPR2 |=  (0x7UL << 3);

    /* Single conversion on channel 1 */
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 1U;

    /* Software trigger (EXTSEL = 111 SWSTART, EXTTRIG = 1), right-aligned */
    ADC1->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_ADON;
    delay_us(20);

    /* Reset and run internal calibration */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    timeout = ADC_TIMEOUT_CYCLES;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) && timeout--) {}

    ADC1->CR2 |= ADC_CR2_CAL;
    timeout = ADC_TIMEOUT_CYCLES;
    while ((ADC1->CR2 & ADC_CR2_CAL) && timeout--) {}
}

uint16_t adc_battery_read_raw(void)
{
    uint32_t sum = 0;
    const uint8_t samples = 8;

    for (uint8_t i = 0; i < samples; i++)
    {
        uint32_t timeout = ADC_TIMEOUT_CYCLES;
        ADC1->SR &= ~ADC_SR_EOC;
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC) && timeout--) {}
        sum += (ADC1->DR & 0x0FFFU);
    }

    return (uint16_t)(sum / samples);
}

uint16_t adc_battery_get_mv(void)
{
    uint32_t raw = adc_battery_read_raw();
    return (uint16_t)((raw * ADC_VREF_MV * BAT_DIVIDER_RATIO) / 4095UL);
}

uint8_t adc_battery_get_percent(void)
{
    uint16_t mv = adc_battery_get_mv();
    if (mv <= BAT_EMPTY_MV) return 0;
    if (mv >= BAT_FULL_MV)  return 100;
    return (uint8_t)(((uint32_t)(mv - BAT_EMPTY_MV) * 100UL) / (BAT_FULL_MV - BAT_EMPTY_MV));
}
