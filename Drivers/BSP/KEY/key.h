/**
 ****************************************************************************************************
 * @file        key.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-01-01
 * @brief       按键输入 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20240101
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef KEY_KEY_H_
#define KEY_KEY_H_
#include "../../SYSTEM/sys/sys.h"

#define KEY0        HAL_GPIO_ReadPin(KEYSHOT_GPIO_Port, KEYSHOT_Pin)   /* C3M1 快门键 PF2 */
#define KEY1        HAL_GPIO_ReadPin(KEYISO_GPIO_Port, KEYISO_Pin)     /* C3M1 ISO键 PF1 */
#define WK_UP       (GPIO_PIN_RESET)                                   /* 量产板无 WK_UP */

#define KEY0_PRES    1              /* KEY0按下 */
#define KEY1_PRES    2              /* KEY1按下 */
#define WKUP_PRES    3              /* KEY_UP按下(即WK_UP) */

uint8_t key_scan(uint8_t mode);     /* 按键扫描函数 */

#endif /* KEY_KEY_H_ */
