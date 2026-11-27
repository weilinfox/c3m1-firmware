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

/*
 * Ultra-low-power single-shot battery voltage sample at 8 MHz HSI (SysTick suspended):
 *   - Runs ADCCLK = 8 MHz / 2 = 4 MHz (no HSE/PLL spin-up, no SysTick delay dependency)
 *   - 239.5-cycle sample time (~60 us > 17.1 us VREFINT min), total burst ~1.1 ms
 *   - Immediately powers down ADC1 + VREFINT + ADC1 clock before returning
 */
uint16_t adc_battery_sample_hsi8_mv(void)
{
    uint32_t timeout;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV2;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;

    /* CH1 (PA1) & CH17 (VREFINT): 239.5 cycles */
    ADC1->SMPR2 = (ADC1->SMPR2 & ~(0x7UL << 3))  | (0x7UL << 3);
    ADC1->SMPR1 = (ADC1->SMPR1 & ~(0x7UL << 21)) | (0x7UL << 21);
    ADC1->SQR1  = 0U;
    ADC1->SQR2  = 0U;
    ADC1->SQR3  = 1U;

    ADC1->CR2 = ADC_CR2_TSVREFE | ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_ADON;
    for (volatile uint32_t d = 0UL; d < 120UL; d++) { __NOP(); } /* ~60 us @ 8 MHz HSI */

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    timeout = ADC_TIMEOUT_CYCLES;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) && timeout--) {}

    ADC1->CR2 |= ADC_CR2_CAL;
    timeout = ADC_TIMEOUT_CYCLES;
    while ((ADC1->CR2 & ADC_CR2_CAL) && timeout--) {}

    uint16_t mv = adc_battery_get_mv();

    ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_TSVREFE);
    RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    return mv;
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

/* Lightest-load (~70 mA sleep peek, LCD/OV7725 off) curve calibrated from steady-work -> sleep pairs:
 * Hardware measurements reveal the classic Li-ion U-shaped DC internal resistance profile:
 *   - High end (90-100%, >=3.91V work): +50 mV (4.00->4.05, 3.98->4.03; 3.91->3.94 rebounds +20 mV -> +50 mV)
 *   - Mid plateau (30-80%, 3.50-3.84V): +30 mV (3.82->3.85, 3.79->3.82, 3.69->3.72, 3.66->3.69, 3.59->3.62, 3.54->3.57, 3.50->3.53)
 *   - Transition (20%, 3.49V): +40 mV -> 3.53V (aligns 3.45/3.50V yellow and 3.44/3.49V red boundaries)
 *   - Low end (0-10%, <=3.45V work): +50 mV (3.45->3.50, 3.43->3.48, 3.41->3.46, 3.30->3.35, 3.25->3.31)
 * Aligns working % and sleep % within <=1% (0% median error) across all 24 measured points. */
uint8_t adc_mv_to_percent_light(uint16_t mv)
{
    static const uint16_t s_light_mv_tbl[11] = {
        3300, 3430, 3530, 3580, 3620, 3660, 3710, 3770, 3850, 3960, 4050
    };
    return adc_tbl_to_percent(mv, s_light_mv_tbl);
}

uint8_t adc_battery_get_percent(void)
{
    return adc_mv_to_percent(adc_battery_get_mv());
}
