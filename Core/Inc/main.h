/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : C3M1 (C3MarkI) production hardware pin definitions.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(void);

/* ======================== C3M1 Production Pinout ======================== */
/* Top board buttons (external 10k pull-up, active LOW) */
#define KEYISO_Pin                  GPIO_PIN_1
#define KEYISO_GPIO_Port            GPIOF
#define KEYSHOT_Pin                 GPIO_PIN_2
#define KEYSHOT_GPIO_Port           GPIOF
#define KEY4X_Pin                   GPIO_PIN_3
#define KEY4X_GPIO_Port             GPIOF
#define FILMKEY_Pin                 GPIO_PIN_10
#define FILMKEY_GPIO_Port           GPIOF

/* Top board LCD control */
#define LCD_BL_Pin                  GPIO_PIN_4      /* LCDEN: active HIGH */
#define LCD_BL_GPIO_Port            GPIOF
#define LCD_RST_Pin                 GPIO_PIN_5      /* LCDRST: active LOW */
#define LCD_RST_GPIO_Port           GPIOF

/* Top board 16-position BCD shutter speed dial (common pulled UP, MCU pull-down, active HIGH) */
#define TSR1_Pin                    GPIO_PIN_6
#define TSR1_GPIO_Port              GPIOF
#define TSR2_Pin                    GPIO_PIN_7
#define TSR2_GPIO_Port              GPIOF
#define TSR4_Pin                    GPIO_PIN_8
#define TSR4_GPIO_Port              GPIOF
#define TSR8_Pin                    GPIO_PIN_9
#define TSR8_GPIO_Port              GPIOF

/* Mainboard ADC, shutter counter reset, SD detect, status LED */
#define BAT_ADC_Pin                 GPIO_PIN_1      /* ADC1_IN1: 10k/10k battery divider */
#define BAT_ADC_GPIO_Port           GPIOA
#define NUMRST_Pin                  GPIO_PIN_3      /* External 10k pull-up, active LOW */
#define NUMRST_GPIO_Port            GPIOA
#define SDOK_Pin                    GPIO_PIN_7      /* External 47k pull-up: 0=present, 1=absent */
#define SDOK_GPIO_Port              GPIOB
#define LED1_Pin                    GPIO_PIN_9      /* Top board red LED, active LOW */
#define LED1_GPIO_Port              GPIOD

/* FSMC 8-bit 8080 LCD control (D0..D3=PD14/15/0/1, D4..D7=PE7..10, CS=GND on top board) */
#define LCD_RS_Pin                  GPIO_PIN_0      /* FSMC_A10 */
#define LCD_RS_GPIO_Port            GPIOG
#define LCD_RD_Pin                  GPIO_PIN_4      /* FSMC_NOE */
#define LCD_RD_GPIO_Port            GPIOD
#define LCD_WR_Pin                  GPIO_PIN_5      /* FSMC_NWE */
#define LCD_WR_GPIO_Port            GPIOD

/* SDIO (4-bit) */
#define SD_D0_Pin                   GPIO_PIN_8
#define SD_D0_GPIO_Port             GPIOC
#define SD_CLK_Pin                  GPIO_PIN_12
#define SD_CLK_GPIO_Port            GPIOC
#define SD_CMD_Pin                  GPIO_PIN_2
#define SD_CMD_GPIO_Port            GPIOD

/* OV7725 + AL422B FIFO control & PG1..PG8 data bus */
#define OV7725_VSYNC_Pin            GPIO_PIN_8
#define OV7725_VSYNC_GPIO_Port      GPIOA
#define OV7725_VSYNC_EXTI_IRQn      EXTI9_5_IRQn
#define SCCB_SCL_Pin                GPIO_PIN_3
#define SCCB_SCL_GPIO_Port          GPIOD
#define OV7725_WRST_Pin             GPIO_PIN_6
#define OV7725_WRST_GPIO_Port       GPIOD
#define FIFO_WE_Pin                 GPIO_PIN_10     /* Direct /WE monitor/override (default Hi-Z input) */
#define FIFO_WE_GPIO_Port           GPIOD
#define FIFO_OE_BUF_Pin             GPIO_PIN_11     /* U7 SN74LVC1G125 OE#: LOW enables NAND->/WE */
#define FIFO_OE_BUF_GPIO_Port       GPIOD
#define SCCB_SDA_Pin                GPIO_PIN_13
#define SCCB_SDA_GPIO_Port          GPIOG
#define OV7725_RRST_Pin             GPIO_PIN_14
#define OV7725_RRST_GPIO_Port       GPIOG
#define OV7725_OE_Pin               GPIO_PIN_15
#define OV7725_OE_GPIO_Port         GPIOG
#define OV7725_WEN_Pin              GPIO_PIN_3
#define OV7725_WEN_GPIO_Port        GPIOB
#define OV7725_RCLK_Pin             GPIO_PIN_4
#define OV7725_RCLK_GPIO_Port       GPIOB

/* Shared modal power watchdog (low-battery < 3.20V forced sleep + auto-sleep) */
uint8_t c3m1_power_service_modal(uint32_t *last_act_tick);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
