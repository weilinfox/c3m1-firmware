/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdio.c
  * @brief   This file provides code for the configuration
  *          of the SDIO instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "sdio.h"
#include "c3m1_log.h"

/* USER CODE BEGIN 0 */
uint8_t g_sd_sta = 0;

#define C3M1_SDIO_CLKDIV_FAST      0x02U /* 72 MHz / (2 + 2) = 18 MHz (4-bit default speed <= 25 MHz) */
#define C3M1_SDIO_CLKDIV_FALLBACK  0x06U /* 72 MHz / (6 + 2) =  9 MHz conservative fallback */

static uint32_t sdio_buf_checksum(const uint32_t *words, uint32_t count)
{
  uint32_t h = 2166136261UL;
  for (uint32_t i = 0U; i < count; i++)
  {
    h ^= words[i];
    h *= 16777619UL;
  }
  return h;
}

static uint8_t sdio_try_init_div(uint32_t clk_div, uint8_t verify_read)
{
  if (hsd.Instance == SDIO && hsd.State != HAL_SD_STATE_RESET)
  {
    HAL_SD_DeInit(&hsd);
  }

  /* 先以 1-bit 初始化，再切 4-bit 总线；开启 ClockPowerSave 空闲自动关 SDIO_CK */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = clk_div;

  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    HAL_SD_DeInit(&hsd);
    return 2U;
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    HAL_SD_DeInit(&hsd);
    return 3U;
  }

  if (verify_read)
  {
    uint32_t probe_buf[128];
    if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)probe_buf, 0U, 1U, 200U) != HAL_OK)
    {
      HAL_SD_DeInit(&hsd);
      return 4U;
    }
    uint32_t h1 = sdio_buf_checksum(probe_buf, 128U);
    if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)probe_buf, 0U, 1U, 200U) != HAL_OK ||
        sdio_buf_checksum(probe_buf, 128U) != h1)
    {
      HAL_SD_DeInit(&hsd);
      return 4U;
    }
  }

  return 0U;
}
/* USER CODE END 0 */

SD_HandleTypeDef hsd;

/* SDIO init function */

uint8_t sd_card_is_present(void)
{
  return (HAL_GPIO_ReadPin(SDOK_GPIO_Port, SDOK_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

void MX_SDIO_SD_DeInit(void)
{
  if (hsd.Instance == SDIO)
  {
    HAL_SD_DeInit(&hsd);
  }
  g_sd_sta = 1;
}

uint8_t MX_SDIO_SD_Init(void)
{
  if (!sd_card_is_present())
  {
    if (hsd.Instance == SDIO && hsd.State != HAL_SD_STATE_RESET)
    {
      HAL_SD_DeInit(&hsd);
    }
    g_sd_sta = 1;
    return 1; /* 无卡，不进入死循环 */
  }

  /* Prefer 18 MHz 4-bit SDIO (with 2x LBA0 CRC+match probe); auto-fallback to 9 MHz on marginal cards */
  uint8_t rc = sdio_try_init_div(C3M1_SDIO_CLKDIV_FAST, 1U);
  if (rc == 0U)
  {
    C3M1_LOG("SDIO: 4-bit 18MHz OK\r\n");
    g_sd_sta = 0;
    return 0;
  }

  C3M1_LOG("SDIO: 18MHz probe err=%u, fallback 9MHz\r\n", (unsigned int)rc);
  rc = sdio_try_init_div(C3M1_SDIO_CLKDIV_FALLBACK, 0U);
  g_sd_sta = rc;
  return rc;
}


void HAL_SD_MspInit(SD_HandleTypeDef* sdHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(sdHandle->Instance==SDIO)
  {
  /* USER CODE BEGIN SDIO_MspInit 0 */

  /* USER CODE END SDIO_MspInit 0 */
    /* SDIO clock enable */
    __HAL_RCC_SDIO_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    GPIO_InitStruct.Pin = SD_D0_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |SD_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SD_CMD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CMD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SDIO_MspInit 1 */

  /* USER CODE END SDIO_MspInit 1 */
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef* sdHandle)
{

  if(sdHandle->Instance==SDIO)
  {
  /* USER CODE BEGIN SDIO_MspDeInit 0 */

  /* USER CODE END SDIO_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDIO_CLK_DISABLE();

    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    HAL_GPIO_DeInit(GPIOC, SD_D0_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |SD_CLK_Pin);

    HAL_GPIO_DeInit(SD_CMD_GPIO_Port, SD_CMD_Pin);

  /* USER CODE BEGIN SDIO_MspDeInit 1 */

  /* USER CODE END SDIO_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief       获取卡信息函数
 * @param       cardinfo:SD卡信息句柄
 * @retval      返回值:读取卡信息状态值
 */
uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo)
{
    uint8_t sta;
    sta = HAL_SD_GetCardInfo(&hsd, cardinfo);
    return sta;
}

/**
 * @brief       判断SD卡是否可以传输(读写)数据
 * @param       无
 * @retval      返回值:SD_TRANSFER_OK      传输完成，可以继续下一次传输
                       SD_TRANSFER_BUSY SD 卡正忙，不可以进行下一次传输
 */
uint8_t get_sd_card_state(void)
{
    return ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
 * @brief       读SD卡(fatfs/usb调用)
 * @param       pbuf  : 数据缓存区
 * @param       saddr : 扇区地址
 * @param       cnt   : 扇区个数
 * @retval      0, 正常;  其他, 错误代码(详见SD_Error定义);
 */
uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint8_t sta = HAL_OK;
    uint32_t timeout = SD_TIMEOUT;

    sta = HAL_SD_ReadBlocks(&hsd, (uint8_t *)pbuf, saddr, cnt, SD_TIMEOUT);

    /* 等待SD卡读完 */
    while (sta == HAL_OK && get_sd_card_state() != SD_TRANSFER_OK)
    {
        if (timeout-- == 0)
        {
            sta = SD_TRANSFER_BUSY;
            break;
        }
    }
    return sta;
}

/**
 * @brief       写SD卡(fatfs/usb调用)
 * @param       pbuf  : 数据缓存区
 * @param       saddr : 扇区地址
 * @param       cnt   : 扇区个数
 * @retval      0, 正常;  其他, 错误代码(详见SD_Error定义);
 */
uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint8_t sta = HAL_OK;
    uint32_t timeout = SD_TIMEOUT;

    sta = HAL_SD_WriteBlocks(&hsd, (uint8_t *)pbuf, saddr, cnt, SD_TIMEOUT);

    /* 等待SD卡写完 */
    while (sta == HAL_OK && get_sd_card_state() != SD_TRANSFER_OK)
    {
        if (timeout-- == 0)
        {
            sta = SD_TRANSFER_BUSY;
            break;
        }
    }
    return sta;
}

/* USER CODE END 1 */
