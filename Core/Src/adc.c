/**
 * @file    adc.c
 * @brief   C3M1 battery voltage ADC1 driver using CMSIS registers (PA1 / ADC1_IN1 + VREFINT CH17).
 *          Hardware: Vbat -> R17(10k) -> PA1 (C16=100nF) -> R18(10k) -> GND.
 */

#include "adc.h"
#include "../../SYSTEM/delay/delay.h"

#define ADC_TIMEOUT_CYCLES  50000U
#define ADC_VREFINT_MV      1200UL  /* STM32F103 internal VREFINT typical = 1.20V */
#define ADC_FALLBACK_VDD_MV 3220UL  /* Measured C3M1 3.3V LDO fallback if VREFINT out of range */
#define BAT_DIVIDER_RATIO   2UL     /* 10k / 10k voltage divider */

void MX_ADC1_Init(void)
{
    uint32_t timeout;

    /* ADCCLK = PCLK2 / 6 = 72MHz / 6 = 12MHz (STM32F103 max is 14MHz) */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Reset ADC1 control registers */
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    /* Channel 1 (PA1): 239.5 cycles for stable reading over 10k divider */
    ADC1->SMPR2 &= ~(0x7UL << 3);
    ADC1->SMPR2 |=  (0x7UL << 3);

    /* Channel 17 (VREFINT): 239.5 cycles (~20us > 17.1us datasheet min) */
    ADC1->SMPR1 &= ~(0x7UL << 21);
    ADC1->SMPR1 |=  (0x7UL << 21);

    /* Single conversion length = 1 */
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 1U;

    /* Enable internal VREFINT + software trigger (SWSTART), right-aligned, ADC ON */
    ADC1->CR2 = ADC_CR2_TSVREFE | ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_ADON;
    delay_us(30);

    /* Reset and run internal ADC self-calibration */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    timeout = ADC_TIMEOUT_CYCLES;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) && timeout--) {}

    ADC1->CR2 |= ADC_CR2_CAL;
    timeout = ADC_TIMEOUT_CYCLES;
    while ((ADC1->CR2 & ADC_CR2_CAL) && timeout--) {}
}

static uint16_t adc_read_channel_avg(uint8_t ch, uint8_t samples)
{
    uint32_t sum = 0;

    ADC1->SQR3 = (uint32_t)(ch & 0x1FU);

    /* Discard first conversion after channel mux switch for clean S&H settling */
    uint32_t timeout = ADC_TIMEOUT_CYCLES;
    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC) && timeout--) {}
    (void)ADC1->DR;

    for (uint8_t i = 0; i < samples; i++)
    {
        timeout = ADC_TIMEOUT_CYCLES;
        ADC1->SR &= ~ADC_SR_EOC;
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC) && timeout--) {}
        sum += (ADC1->DR & 0x0FFFU);
    }

    return (uint16_t)(sum / samples);
}

uint16_t adc_battery_read_raw(void)
{
    return adc_read_channel_avg(1U, 8U);
}

uint16_t adc_battery_get_mv(void)
{
    uint32_t raw_bat  = adc_read_channel_avg(1U, 8U);
    uint32_t raw_vref = adc_read_channel_avg(17U, 8U);

    /* Self-calibrated against STM32 1.20V VREFINT (cancels out LDO voltage & temp drift) */
    if (raw_vref >= 1200UL && raw_vref <= 1850UL)
    {
        return (uint16_t)((raw_bat * ADC_VREFINT_MV * BAT_DIVIDER_RATIO + (raw_vref / 2UL)) / raw_vref);
    }
    return (uint16_t)((raw_bat * ADC_FALLBACK_VDD_MV * BAT_DIVIDER_RATIO) / 4095UL);
}

static uint8_t adc_tbl_to_percent(uint16_t mv, const uint16_t tbl[11])
{
    if (mv <= tbl[0])  return 0U;
    if (mv >= tbl[10]) return 100U;

    for (uint8_t i = 0; i < 10U; i++)
    {
        if (mv <= tbl[i + 1U])
        {
            uint32_t span = (uint32_t)(tbl[i + 1U] - tbl[i]);
            uint32_t frac = ((uint32_t)(mv - tbl[i]) * 10UL) / span;
            return (uint8_t)(i * 10U + frac);
        }
    }
    return 100U;
}

/* 10-segment single-cell Li-ion under-load (~400 mA C3M1 viewfinder) discharge curve:
 * 4.00V = 100% (unplugged full charge after initial surface-charge drop), 3.25V = 0% (SD write lockout). */
uint8_t adc_mv_to_percent(uint16_t mv)
{
    static const uint16_t s_mv_tbl[11] = {
        3250, 3380, 3490, 3550, 3590, 3630, 3680, 3740, 3820, 3910, 4000
    };
    return adc_tbl_to_percent(mv, s_mv_tbl);
}

/* Lightest-load (~70 mA sleep peek, LCD/OV7725 off) curve aligned to working curve:
 * Hardware measurements across full discharge range (4.00->4.05V, 3.43->3.48V, 3.41->3.46V, 3.30->3.35V,
 * 3.25->3.30-3.31V) confirm a constant +50 mV ohmic offset from 100% down to 0% (3.30V..4.05V). */
#define BAT_SLEEP_OFFSET_MV 50U

uint8_t adc_mv_to_percent_light(uint16_t mv)
{
    uint16_t eq_work_mv = (mv > BAT_SLEEP_OFFSET_MV) ? (uint16_t)(mv - BAT_SLEEP_OFFSET_MV) : 0U;
    return adc_mv_to_percent(eq_work_mv);
}

uint8_t adc_battery_get_percent(void)
{
    return adc_mv_to_percent(adc_battery_get_mv());
}
