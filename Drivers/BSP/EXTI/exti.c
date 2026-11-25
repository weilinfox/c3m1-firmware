/**
 ****************************************************************************************************
 * @file        exti.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-19
 * @brief       外部中断 驱动代码
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
 * V1.0 20200420
 * 第一次发布
 * V1.1 20200426
 * 1, 增加exti8_init函数及对应中断服务函数
 * 2, 支持对OV7725 VSYNC中断的处理
 * 3, 增加 g_ov7725_vsta 全局变量
 *
 ****************************************************************************************************
 */

#include "../../SYSTEM/sys/sys.h"
#include "../../SYSTEM/delay/delay.h"
#include "exti.h"
#include "../../BSP/OV7725/ov7725.h"

/* OV7725 帧中断标志*/
volatile uint8_t  g_vsync_state     = 0; /*定义FIFO状态标志位，0为空闲，1为写中，2为写完就绪*/
volatile uint32_t g_last_vsync_tick = 0; /*最近一次 VSYNC 上升沿时间戳(ms), 供主循环避开VSYNC边界写SCCB*/

/** @brief       OV7725 VSYNC 外部中断服务程序*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)   /*帧同步信号触发的中断回调函数，上升沿触发*/
{
    if (GPIO_Pin == OV7725_VSYNC_Pin)   /* 复核一次，确实是中断来了，处于帧起始处的高电平 */
    {
        g_last_vsync_tick = HAL_GetTick();
    	switch (g_vsync_state) /*通过标志位判断当前FIFO所处状态*/
    	{
    		case 0:  // 情况0：当前空闲（第1次VSYNC触发）→ 允许写入新帧
			OV7725_WRST(0);     // 写指针复位到0
			for (volatile uint8_t d = 0; d < 12U; d++) { __NOP(); }
			OV7725_WRST(1);         // 结束复位
			OV7725_WEN(1);          // 允许写入FIFO   /*以上几步在实际内容来之前已经做完*/
			g_vsync_state = 1; 		// 标记为FIFO“写中”
			break;

			case 1:  // 情况1：第2次VSYNC触发，说明上一帧已写入FIFO完毕，FIFO读有关设置归零并开启，标记为MCU可读
			OV7725_WEN(0);          // 禁止写入FIFO（避免新帧覆盖）
			g_vsync_state = 2; 		// 标记为“FIFO写完完整一帧现在可读”
			break;

    	    case 2: // 情况2：当前在读但MCU未读完，无视，直接break
    	    OV7725_WEN(0);              // 保持禁止写，避免覆盖
    	    break;

    	    default:
    	    g_vsync_state = 0; // 异常状态重置
    	    break;
    	}
    	    // HAL库会自动清除中断标志位，无需手动重复清
    }
}
