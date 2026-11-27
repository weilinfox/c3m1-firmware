/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdio.h
  * @brief   This file contains all the function prototypes for
  *          the sdio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDIO_H__
#define __SDIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SD_HandleTypeDef hsd;
extern uint8_t g_sd_sta;

/* USER CODE BEGIN Private defines */

#define SD_TIMEOUT             ((uint32_t)2000U)        /* HAL_GetTick() ms timeout for SDIO polling read/write */
#define SD_TRANSFER_OK         ((uint8_t)0x00)        /* 表示SD卡操作（读/写/初始化）成功完成 */
#define SD_TRANSFER_BUSY       ((uint8_t)0x01)        /* 表示SD卡正在忙（比如写数据时，SD卡内部正在处理数据，未响应主机）*/

/* 根据 SD_HandleTypeDef 定义的宏，用于快速计算容量 */
/*无需手动计算 SD 卡容量，直接调用宏即可。比如判断 SD 卡是否够存 DNG*/
#define SD_TOTAL_SIZE_BYTE(__Handle__)  (((uint64_t)((__Handle__)->SdCard.LogBlockNbr)*((__Handle__)->SdCard.LogBlockSize))>>0)
#define SD_TOTAL_SIZE_KB(__Handle__)    (((uint64_t)((__Handle__)->SdCard.LogBlockNbr)*((__Handle__)->SdCard.LogBlockSize))>>10)
#define SD_TOTAL_SIZE_MB(__Handle__)    (((uint64_t)((__Handle__)->SdCard.LogBlockNbr)*((__Handle__)->SdCard.LogBlockSize))>>20)
#define SD_TOTAL_SIZE_GB(__Handle__)    (((uint64_t)((__Handle__)->SdCard.LogBlockNbr)*((__Handle__)->SdCard.LogBlockSize))>>30)
/* USER CODE END Private defines */

uint8_t MX_SDIO_SD_Init(void);
void    MX_SDIO_SD_DeInit(void);
uint8_t sd_card_is_present(void);

/* USER CODE BEGIN Prototypes */

uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo);          /*SD卡基本信息*/
uint8_t get_sd_card_state(void);                                     /*SD卡当前状态*/
uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt);   /*SD卡的扇区读封装函数 */
uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt);  /*SD卡的扇区写封装函数 */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SDIO_H__ */

