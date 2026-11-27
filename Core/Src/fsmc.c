/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fsmc.c
  * @brief   C3M1 8-bit 8080 FSMC configuration for ST7789V3 LCD.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "fsmc.h"

/* FSMC initialization function (Bank1 NE1, 8-bit data width, A10 = LCD DC/RS) */
void MX_FSMC_Init(void)
{
  SRAM_HandleTypeDef hsram1 = {0};
  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* LCD CS is hardwired to GND on C3M1 top board; use Bank1 NE1 (0x60000000) */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;

  /* Read timing (conservative for ST7789V3 ID / GRAM reads) */
  Timing.AddressSetupTime = 1;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 15;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;

  /* Write timing (fast 8080 write: DataSetup=2 HCLK ~27.8ns @72MHz, safe over 24-pin FPC) */
  ExtTiming.AddressSetupTime = 0;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 2;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

static uint32_t FSMC_Initialized = 0;

static void HAL_FSMC_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (FSMC_Initialized)
  {
    return;
  }
  FSMC_Initialized = 1;

  __HAL_RCC_FSMC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /** C3M1 8-bit FSMC GPIO Configuration:
    PG0   ------> FSMC_A10  (LCD_DC / RS)
    PD14  ------> FSMC_D0
    PD15  ------> FSMC_D1
    PD0   ------> FSMC_D2
    PD1   ------> FSMC_D3
    PE7   ------> FSMC_D4
    PE8   ------> FSMC_D5
    PE9   ------> FSMC_D6
    PE10  ------> FSMC_D7
    PD4   ------> FSMC_NOE  (LCD_RD)
    PD5   ------> FSMC_NWE  (LCD_WR)
    NOTE: PD9=LED1, PD10=/WE, PG1..PG8=OV_D0..D7 are NOT configured as FSMC!
  */
  GPIO_InitStruct.Pin = LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
                      | LCD_RD_Pin | LCD_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef *sramHandle)
{
  (void)sramHandle;
  HAL_FSMC_MspInit();
}

static uint32_t FSMC_DeInitialized = 0;

static void HAL_FSMC_MspDeInit(void)
{
  if (FSMC_DeInitialized)
  {
    return;
  }
  FSMC_DeInitialized = 1;
  __HAL_RCC_FSMC_CLK_DISABLE();

  HAL_GPIO_DeInit(GPIOG, LCD_RS_Pin);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1
                       | LCD_RD_Pin | LCD_WR_Pin);
}

void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *sramHandle)
{
  (void)sramHandle;
  HAL_FSMC_MspDeInit();
}
