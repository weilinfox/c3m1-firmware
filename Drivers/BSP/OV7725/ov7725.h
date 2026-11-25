/**
 ****************************************************************************************************
 * @file        ov7725.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-26
 * @brief       OV7725 驱动代码
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
 * V1.0 20200426
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef _OV7725_H
#define _OV7725_H

#include "../../SYSTEM/sys/sys.h"


/******************************************************************************************/
/* OV7725模块的的 VSYNC/WRST/RRST/OE/RCLK/WEN 引脚 定义
 * D0~D7, 由于引脚多, 且连续, 这里就不在这里定义了,直接在ov7725_init里面修改.所以在移植
 * 的时候, 除了改这6个IO口, 还得改ov7725_init里面的D0~D7所在的IO口.
 */
/******************************************************************************************/

/* OV7725相关端口定义 */
#define OV7725_VSYNC(x)   do{ x ? \
                              HAL_GPIO_WritePin(OV7725_VSYNC_GPIO_Port, OV7725_VSYNC_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OV7725_VSYNC_GPIO_Port, OV7725_VSYNC_Pin, GPIO_PIN_RESET); \
                          }while(0)  		/* VSYNC */

#define OV7725_WRST(x)    do{ x ? \
                              HAL_GPIO_WritePin(OV7725_WRST_GPIO_Port, OV7725_WRST_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OV7725_WRST_GPIO_Port, OV7725_WRST_Pin, GPIO_PIN_RESET); \
                          }while(0)    		/* WRST */

#define OV7725_RRST(x)    do{ x ? \
                              HAL_GPIO_WritePin(OV7725_RRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OV7725_RRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_RESET); \
                          }while(0)    		/* RRST */
                          
#define OV7725_OE(x)      do{ x ? \
                              HAL_GPIO_WritePin(OV7725_OE_GPIO_Port, OV7725_OE_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OV7725_OE_GPIO_Port, OV7725_OE_Pin, GPIO_PIN_RESET); \
                          }while(0)        	/* OE */

#define OV7725_RCLK(x)      x ? (OV7725_RCLK_GPIO_Port->BSRR = OV7725_RCLK_Pin) : (OV7725_RCLK_GPIO_Port->BRR = OV7725_RCLK_Pin)    /* RCLK */
//#define OV7725_RCLK(x)      HAL_GPIO_TogglePin(OV7725_RCLK_GPIO_PORT,OV7725_RCLK_GPIO_PIN);

#define OV7725_WEN(x)     do{ x ? \
                              HAL_GPIO_WritePin(OV7725_WEN_GPIO_Port, OV7725_WEN_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OV7725_WEN_GPIO_Port, OV7725_WEN_Pin, GPIO_PIN_RESET); \
                          }while(0)      	/* WEN */

/* C3M1 量产板 OV7725 D0~D7 映射在 PG1~PG8 */
#define OV7725_DATA         ((uint8_t)((GPIOG->IDR >> 1) & 0x00FFU))

/* U7 (SN74LVC1G125) 输出使能控制 (PD11 OE#): 拉低允许 U4(OV_WEN NAND HREF) 驱动 AL422B /WE */
#define FIFO_WE_BUF_ENABLE()   HAL_GPIO_WritePin(FIFO_OE_BUF_GPIO_Port, FIFO_OE_BUF_Pin, GPIO_PIN_RESET)
#define FIFO_WE_BUF_DISABLE()  HAL_GPIO_WritePin(FIFO_OE_BUF_GPIO_Port, FIFO_OE_BUF_Pin, GPIO_PIN_SET)


/* OV7725 ID */
#define OV7725_MID              0X7FA2      /* MID , 第一个MID*/
#define OV7725_MID1             0X7FFF      /* MID1, 第二个MID */
#define OV7725_PID              0X7721

/* OV7670的SCCB地址 */
#define OV7725_ADDR             0X42


//void ov7725_init(void);                             /* OV7725初始化 */
uint8_t ov7725_init(void);

uint8_t ov7725_RAW_init(void);                      /* OV7725RAW初始化测试 */

uint8_t ov7725_read_reg(uint16_t reg);              /* OV7725 读寄存器 */
uint8_t ov7725_write_reg(uint8_t reg, uint8_t data);/* OV7725写寄存器 */

void ov7725_light_mode(uint8_t mode);       /* OV7725 灯光模式设置 */
void ov7725_color_saturation(uint8_t sat);  /* OV7725 色彩饱和度设置 */
void ov7725_brightness(uint8_t bright);     /* OV7725 亮度设置 */
void ov7725_contrast(uint8_t contrast);     /* OV7725 对比度设置 */
void ov7725_special_effects(uint8_t eft);   /* OV7725 特效设置 */

void ov7725_window_set(uint16_t width, uint16_t height);  /* OV7725 输出窗口设置 */

/* C3M1 手动快门 (16档)、ISO 增益与常驻 RAW 取景曝光控制 */
#define OV7725_ISO_COUNT        6U      /* 0:AUTO, 1:100, 2:200, 3:400, 4:800, 5:1600 */
#define OV7725_SHUTTER_COUNT    16U     /* 0..8: 1/8000..1/50, 9..14: 1/25..2s, 15: AUTO */
#define OV7725_PREVIEW_MAX_AEC  1400U   /* ~1/12s cap to guarantee >=12fps viewfinder */

uint8_t  ov7725_is_slow_shutter(uint8_t dial_pos);
uint8_t  ov7725_is_auto_shutter(uint8_t dial_pos);
uint16_t ov7725_apply_always_raw_preview(uint8_t dial_pos, uint8_t iso_idx, uint16_t auto_aec, uint8_t auto_gain);
void     ov7725_apply_raw_exposure(uint8_t dial_pos, uint8_t iso_idx, uint16_t auto_aec, uint8_t auto_gain);
void     ov7725_format_shutter_label(uint8_t dial_pos, uint16_t auto_aec, char *buf, uint8_t buf_len);
const char *ov7725_shutter_label(uint8_t dial_pos);
const char *ov7725_iso_label(uint8_t iso_idx);
uint8_t  ov7725_iso_to_gain_reg(uint8_t iso_idx);
uint8_t  ov7725_gain_q4_to_reg(uint16_t gain_q4);

/* Legacy preview helpers kept for reference */
void ov7725_set_preview_mode(uint8_t zoom_4x);
void ov7725_apply_exposure(uint8_t dial_pos, uint8_t iso_idx, uint8_t is_raw_mode);
void ov7725_snapshot_auto_exposure(uint16_t *aec_lines, uint8_t *gain_reg);

#endif





















