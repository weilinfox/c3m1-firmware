/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   C3M1 (C3MarkI) production hardware GPIO initialization.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Default output levels before enabling outputs */
  /* LCD backlight OFF initially, LCD reset released HIGH */
  HAL_GPIO_WritePin(GPIOF, LCD_BL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, LCD_RST_Pin, GPIO_PIN_SET);

  /* LED1 OFF (active LOW), SCCB_SCL idle HIGH, WRST idle HIGH, U7 OE# LOW (enable NAND->/WE) */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin | SCCB_SCL_Pin | OV7725_WRST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, FIFO_OE_BUF_Pin, GPIO_PIN_RESET);

  /* FIFO WEN disabled initially (LOW), RCLK idle HIGH */
  HAL_GPIO_WritePin(GPIOB, OV7725_WEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, OV7725_RCLK_Pin, GPIO_PIN_SET);

  /* SCCB_SDA idle HIGH, RRST idle HIGH, OE idle HIGH (disabled) */
  HAL_GPIO_WritePin(GPIOG, SCCB_SDA_Pin | OV7725_RRST_Pin | OV7725_OE_Pin, GPIO_PIN_SET);

  /* ---------------- Top board buttons (external 10k pull-up, active LOW) ---------------- */
  GPIO_InitStruct.Pin = KEYISO_Pin | KEYSHOT_Pin | KEY4X_Pin | FILMKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* ---------------- Top board 16-pos BCD shutter dial (internal pull-down) -------------- */
  GPIO_InitStruct.Pin = TSR1_Pin | TSR2_Pin | TSR4_Pin | TSR8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* ---------------- Top board LCD backlight & reset ---------------- */
  GPIO_InitStruct.Pin = LCD_BL_Pin | LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* ---------------- Mainboard NUMRST (PA3) & Battery ADC (PA1) ---------------- */
  GPIO_InitStruct.Pin = NUMRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NUMRST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BAT_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(BAT_ADC_GPIO_Port, &GPIO_InitStruct);

  /* ---------------- OV7725 VSYNC EXTI (PA8 rising edge) ---------------- */
  GPIO_InitStruct.Pin = OV7725_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(OV7725_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /* ---------------- OV7725 Data Bus D0~D7 (PG1~PG8) ---------------- */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
                      | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* ---------------- OV7725 SCCB & FIFO Control Pins ---------------- */
  GPIO_InitStruct.Pin = SCCB_SCL_Pin | OV7725_WRST_Pin | FIFO_OE_BUF_Pin | LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PD10 (/WE) kept as Hi-Z input so U7 (SN74LVC1G125) drives AL422B /WE cleanly */
  GPIO_InitStruct.Pin = FIFO_WE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FIFO_WE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SCCB_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCCB_SDA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OV7725_RRST_Pin | OV7725_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OV7725_WEN_Pin | OV7725_RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ---------------- SD Card Detect SDOK (PB7, external 47k pull-up, 0=present) ---------------- */
  GPIO_InitStruct.Pin = SDOK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDOK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init for OV7725 VSYNC */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
