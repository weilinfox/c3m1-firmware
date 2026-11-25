/*
 * @file        ov7725.c
 * @brief       OV7725 驱动代码
 */

#include <stdio.h>
#include "sccb.h"
#include "ov7725.h"
#include "ov7725cfg.h"
#include "usart.h"
#include "../../SYSTEM/delay/delay.h"

/**
 * @brief       OV7725 读SCCB寄存器
 * @param       reg : 寄存器地址
 * @retval      读到的寄存器值
 */
/*为什么分两次通信‌？第一次*预约*要读的寄存器，第二次*取数据*，这是协议规定的步骤*/
uint8_t ov7725_read_reg(uint16_t reg)  /*读OV7725的SCCB，要读的寄存器是16位的，返回一个8位结果*/
{
    uint8_t data = 0;  /*先定义一个变量data给初值0，一会用来存读到的数*/
    
    sccb_start();                       /* 通信起始信号，相当于对OV说：我要开始和你说话了！ */
    sccb_send_byte(OV7725_ADDR);        /* 发送通讯地址，就是IIC总线里面的摄像头设备地址比如0X00 */
    sccb_send_byte(reg);                /* 把要读的寄存器地址告诉OV */
    sccb_stop();                        /* 说完上面的话，先停一下。停止信号 */
    
    /* 设置寄存器地址后，才是读 */
    sccb_start();                       /* 起始信号，重新开始通讯 */
    sccb_send_byte(OV7725_ADDR | 0X01); /* 读通讯地址 ，设备地址+1表示要读*/
    data = sccb_read_byte();            /* 读取数据，比如亮度、对比度等等 */
    sccb_nack();                        /* 非应答信号，要终止通信了。好了，我收到了，不用再发了！ */
    sccb_stop();                        /* 停止信号 */
    
    return data;   /*把读到的寄存器值（比如0x80）返回给调用者*/
}

/**
 * @brief       OV7725 写SCCB寄存器
 * @param       reg : 寄存器地址
 * @param       data: 要写入寄存器的值
 * @retval      0, 成功; 1, 失败;
 */
uint8_t ov7725_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t res = 0;
    
    sccb_start();   /* 起始信号 */

    if (sccb_send_byte(OV7725_ADDR))res = 1;    /* 写通讯地址 */
    if (sccb_send_byte(reg))res = 1;            /* 寄存器地址 */
    if (sccb_send_byte(data))res = 1;           /* 写数据 */
    sccb_stop();    /* 停止信号 */
    
    return res;
}

/* 初始化 OV7725 */
//void ov7725_init(void)  /*这个函数是OV7725摄像头的*开机自检+初始化流程*，返回值0表示成功，1或2表示失败。*/
//{
//    uint16_t i = 0;
//    sccb_init(); /* 初始化SCCB的IO口  初始化SCCB总线（就像给摄像头和主控搭个电话线） */
//    ov7725_write_reg(0x12, 0x80);
//    delay_ms(5);
//
//    /*初始化 OV7725,采用QVGA分辨率(320*240),ov7725_init_reg_tb1是一个表格，存了要写的寄存器值和数据（比如分辨率、亮度、对比度）。用循环遍历表格，挨个写入摄像头.*/
//    for (i = 0; i < sizeof(ov7725_init_reg_tb1) / sizeof(ov7725_init_reg_tb1); i++)
//    {
//        ov7725_write_reg(ov7725_init_reg_tb1[i][0], ov7725_init_reg_tb1[i][1]);
//    }
//}

uint8_t ov7725_init(void)
{
    uint16_t i = 0;
    uint16_t reg = 0;

    OV7725_WRST(1);             /* WRST =1 */
    OV7725_RRST(1);             /* RRST =1 */
    OV7725_OE(1);               /* OE =1 */
    OV7725_RCLK(1);             /* RCLK =1 */
    OV7725_WEN(1);              /* WEN =1 */

    sccb_init();                /* 初始化SCCB 的IO口 */

    if (ov7725_write_reg(0x12, 0x80))   /* 复位 */
    {
        return 1;
    }

    delay_ms(50);
    reg = ov7725_read_reg(0X1c);    /* 读取厂家ID 高八位 */
    reg <<= 8;
    reg |= ov7725_read_reg(0X1d);   /* 读取厂家ID 低八位 */

    if ((reg != OV7725_MID) && (reg != OV7725_MID1))    /* MID不正确 ? */
    {
        printf("MID:%d\r\n", reg);
        return 1;
    }

    reg = ov7725_read_reg(0X0a);    /* 读取厂家ID 高八位 */
    reg <<= 8;
    reg |= ov7725_read_reg(0X0b);   /* 读取厂家ID 低八位 */

    if (reg != OV7725_PID)          /* PID不正确 ? */
    {
        printf("HID:%d\r\n", reg);
        return 2;
    }

    /* 初始化 OV7725,采用QVGA分辨率(320*240) */
    for (i = 0; i < sizeof(ov7725_init_reg_tb1) / sizeof(ov7725_init_reg_tb1[0]); i++)
    {
        ov7725_write_reg(ov7725_init_reg_tb1[i][0], ov7725_init_reg_tb1[i][1]);
    }

    return 0;   /* ok */
}


/*这段代码用于测试RAW格式*/
uint8_t ov7725_RAW_init(void)  /*这个函数是OV7725摄像头RAW输出的*开机自检+初始化流程*，返回值0表示成功，1或2表示失败。*/
{
    uint16_t i = 0;
    sccb_init(); /* 初始化SCCB 的IO口  初始化SCCB总线（就像给摄像头和主控搭个*电话线*） */
    ov7725_write_reg(0x12, 0x80);/*重置寄存器*/
    delay_ms(2);
    /*ov7725_init_reg_tb1是一个表格，存了数据手册中一部分要写的寄存器值和数据（比如分辨率、亮度、对比度）。用循环遍历表格，挨个写入摄像头，相当于*开机自检完成，开始干活！**/
    for (i = 0; i < sizeof(ov7725_init_reg_tbRAW) / sizeof(ov7725_init_reg_tbRAW[0]); i++)
    {
        ov7725_write_reg(ov7725_init_reg_tbRAW[i][0], ov7725_init_reg_tbRAW[i][1]);
    }
    return 0;   /* ok */
}

/**
 * @brief       设置图像输出窗口
 * @param       width   : 输出图像宽度, <=320
 * @param       height  : 输出图像高度, <=240
 * @param       mode    : 输出模式
 *   @arg       0: QVGA输出模式, 可视范围大, 最大输出分辨率: 320*240
 *   @arg       1: VGA输出模式 , 可视范围小, 最大输出分辨率: 320*240
 * @retval      无
 */
void ov7725_window_set(uint16_t width, uint16_t height)
{
    uint8_t raw, temp;
    uint16_t sx, sy;

	sx = (320 - width) / 2;
	sy = (240 - height) / 2;

	ov7725_write_reg(COM7, 0x46);       /* 设置为QVGA模式 */
	ov7725_write_reg(HSTART, 0x3f);     /* 水平起始位置 */
	ov7725_write_reg(HSIZE, 0x50);      /* 水平尺寸 */
	ov7725_write_reg(VSTRT, 0x03);      /* 垂直起始位置 */
	ov7725_write_reg(VSIZE, 0x78);      /* 垂直尺寸 */
	ov7725_write_reg(HREF,  0x00);
	ov7725_write_reg(HOutSize, 0x50);   /* 输出尺寸 */
	ov7725_write_reg(VOutSize, 0x78);   /* 输出尺寸 */

    raw = ov7725_read_reg(HSTART);
    temp = raw + (sx >> 2);                 /* sx高8位存在HSTART,低2位存在HREF[5:4] */
    ov7725_write_reg(HSTART, temp);
    ov7725_write_reg(HSIZE, width >> 2);    /* width高8位存在HSIZE,低2位存在HREF[1:0] */

    raw = ov7725_read_reg(VSTRT);
    temp = raw + (sy >> 1);                 /* sy高8位存在VSTRT,低1位存在HREF[6] */
    ov7725_write_reg(VSTRT, temp);
    ov7725_write_reg(VSIZE, height >> 1);   /* height高8位存在VSIZE,低1位存在HREF[2] */

    raw = ov7725_read_reg(HREF);
    temp = ((sy & 0x01) << 6) | ((sx & 0x03) << 4) | ((height & 0x01) << 2) | (width & 0x03) | raw;
    ov7725_write_reg(HREF, temp);

    ov7725_write_reg(HOutSize, width >> 2);
    ov7725_write_reg(VOutSize, height >> 1);

    ov7725_read_reg(EXHCH);
    temp = (raw | (width & 0x03) | ((height & 0x01) << 2));
    ov7725_write_reg(EXHCH, temp);
}

/**
 * @brief       OV7725 灯光模式设置
 * @param       mode : 0~5, 灯光模式
 *   @arg       0: 自动
 *   @arg       1: 日光sunny
 *   @arg       2: 阴天cloudy
 *   @arg       3: 办公室office
 *   @arg       4: 家里home
 *   @arg       5: 夜晚night
 * @retval      无
 */
void ov7725_light_mode(uint8_t mode)
{
    switch (mode)
    {
        case 0: /* auto, 自动模式 */
            ov7725_write_reg(0x13, 0xff); /* AWB on */
            ov7725_write_reg(0x0e, 0x65);
            ov7725_write_reg(0x2d, 0x00);
            ov7725_write_reg(0x2e, 0x00);
            break;

        case 1: /* sunny, 晴天 */
            ov7725_write_reg(0x13, 0xfd); /* AWB off */
            ov7725_write_reg(0x01, 0x5a);
            ov7725_write_reg(0x02, 0x5c);
            ov7725_write_reg(0x0e, 0x65);
            ov7725_write_reg(0x2d, 0x00);
            ov7725_write_reg(0x2e, 0x00);
            break;

        case 2: /* cloudy, 阴天 */
            ov7725_write_reg(0x13, 0xfd); /* AWB off */
            ov7725_write_reg(0x01, 0x58);
            ov7725_write_reg(0x02, 0x60);
            ov7725_write_reg(0x0e, 0x65);
            ov7725_write_reg(0x2d, 0x00);
            ov7725_write_reg(0x2e, 0x00);
            break;

        case 3: /* office, 办公室 */
            ov7725_write_reg(0x13, 0xfd); /* AWB off */
            ov7725_write_reg(0x01, 0x84);
            ov7725_write_reg(0x02, 0x4c);
            ov7725_write_reg(0x0e, 0x65);
            ov7725_write_reg(0x2d, 0x00);
            ov7725_write_reg(0x2e, 0x00);
            break;

        case 4: /* home, 家里 */
            ov7725_write_reg(0x13, 0xfd); /* AWB off */
            ov7725_write_reg(0x01, 0x96);
            ov7725_write_reg(0x02, 0x40);
            ov7725_write_reg(0x0e, 0x65);
            ov7725_write_reg(0x2d, 0x00);
            ov7725_write_reg(0x2e, 0x00);
            break;

        case 5: /* night, 夜晚 */
            ov7725_write_reg(0x13, 0xff); /* AWB on */
            ov7725_write_reg(0x0e, 0xe5);
            break;
    }
}

/**
 * @brief       OV7725 色彩饱和度设置
 * @param       set : 0~8, 代表色彩饱和度 -4 ~ 4.
 * @retval      无
 */
void ov7725_color_saturation(uint8_t sat)
{
    ov7725_write_reg(USAT, sat << 4);
    ov7725_write_reg(VSAT, sat << 4);
}

/**
 * @brief       OV7725 亮度设置
 * @param       bright : 0~8, 代表亮度 -4 ~ 4.
 * @retval      无
 */
void ov7725_brightness(uint8_t bright)
{
    uint8_t bright_value = 0, sign = 0;

    switch (bright)
    {
        case 8:
            bright_value = 0x48;
            sign = 0x06;
            break;

        case 7:
            bright_value = 0x38;
            sign = 0x06;
            break;

        case 6:
            bright_value = 0x28;
            sign = 0x06;
            break;

        case 5:
            bright_value = 0x18;
            sign = 0x06;
            break;

        case 4:
            bright_value = 0x08;
            sign = 0x06;
            break;

        case 3:
            bright_value = 0x08;
            sign = 0x0e;
            break;

        case 2:
            bright_value = 0x18;
            sign = 0x0e;
            break;

        case 1:
            bright_value = 0x28;
            sign = 0x0e;
            break;

        case 0:
            bright_value = 0x38;
            sign = 0x0e;
            break;
    }

    ov7725_write_reg(BRIGHT, bright_value);
    ov7725_write_reg(SIGN, sign);
}

/**
 * @brief       OV7725 对比度设置
 * @param       contrast : 0~8, 代表对比度 -4 ~ 4.
 * @retval      无
 */
void ov7725_contrast(uint8_t contrast)
{
    ov7725_write_reg(CNST, (0x30 - (8 - contrast) * 4));
}

/**
 * @brief       OV7725 特效设置
 * @param       eft : 0~6, 特效效果
 *   @arg       0: 正常
 *   @arg       1: 负片
 *   @arg       2: 黑白
 *   @arg       3: 偏红
 *   @arg       4: 偏绿
 *   @arg       5: 偏蓝
 *   @arg       6: 复古
 * @retval      无
 */
void ov7725_special_effects(uint8_t eft)
{
    switch (eft)
    {
        case 0: /* 正常 */
            ov7725_write_reg(0xa6, 0x06);   /* TSLB设置 */
            ov7725_write_reg(0x60, 0x80);   /* MANV,手动V值 */
            ov7725_write_reg(0x61, 0x80);   /* MANU,手动U值 */
            break;

        case 1: /* 负片 */
            ov7725_write_reg(0xa6, 0x46);
            break;

        case 2: /* 黑白 */
            ov7725_write_reg(0xa6, 0x26);
            ov7725_write_reg(0x60, 0x80);
            ov7725_write_reg(0x61, 0x80);
            break;

        case 3: /* 偏红 */
            ov7725_write_reg(0xa6, 0x1e);
            ov7725_write_reg(0x60, 0x80);
            ov7725_write_reg(0x61, 0xc0);
            break;

        case 4: /* 偏绿 */
            ov7725_write_reg(0xa6, 0x1e);
            ov7725_write_reg(0x60, 0x60);
            ov7725_write_reg(0x61, 0x60);
            break;

        case 5: /* 偏蓝 */
            ov7725_write_reg(0xa6, 0x1e);
            ov7725_write_reg(0x60, 0xa0);
            ov7725_write_reg(0x61, 0x40);
            break;

        case 6: /* 复古 */
            ov7725_write_reg(0xa6, 0x1e);
            ov7725_write_reg(0x60, 0x40);
            ov7725_write_reg(0x61, 0xa0);
            break;

    }
}

/* ==================== C3M1 曝光/ISO/4X对焦放大控制 ==================== */

/* 16 档快门拨盘映射表:
 *   0..8  = 机顶物理刻度手动档 (1/8000 ~ 1/50, 对齐原厂 0x08001CA8 实测 AECH/AEC)
 *   9..14 = 扩展慢门手动档 (1/25, 1/8, 1/4, 1/2, 1s, 2s, 配合 DM_LNL/DM_LNH 插哑行)
 *   15    = AUTO 自动测光档
 */
#define OV7725_DIAL_IS_AUTO(pos)  ((pos) >= 15U)

static const uint16_t s_shutter_aec_tbl[OV7725_SHUTTER_COUNT] = {
    0x0004, /* 0:  1/8000 */
    0x0009, /* 1:  1/4000 */
    0x000B, /* 2:  1/2000 */
    0x000F, /* 3:  1/1000 */
    0x0014, /* 4:  1/500  */
    0x0028, /* 5:  1/300  */
    0x004B, /* 6:  1/150  */
    0x00E6, /* 7:  1/100  */
    0x0150, /* 8:  1/50   (336 lines) */
    0x02A0, /* 9:  1/25   (672 lines) */
    0x0834, /* 10: 1/8    (2100 lines) */
    0x1068, /* 11: 1/4    (4200 lines) */
    0x20D0, /* 12: 1/2    (8400 lines) */
    0x41A0, /* 13: 1s     (16800 lines) */
    0x8340, /* 14: 2s     (33600 lines) */
    0x0000  /* 15: AUTO   */
};

/* 慢门附加哑行数 (DM_LNH/DM_LNL: 保证总帧行数 510 + dummy > AEC) */
static const uint16_t s_shutter_dummy_tbl[OV7725_SHUTTER_COUNT] = {
    0x0000, /* 0:  1/8000 */
    0x0000, /* 1:  1/4000 */
    0x0000, /* 2:  1/2000 */
    0x0000, /* 3:  1/1000 */
    0x0000, /* 4:  1/500  */
    0x0000, /* 5:  1/300  */
    0x0000, /* 6:  1/150  */
    0x0000, /* 7:  1/100  */
    0x0000, /* 8:  1/50   */
    0x00C8, /* 9:  1/25   (+200 lines) */
    0x0672, /* 10: 1/8    (+1650 lines) */
    0x0EA6, /* 11: 1/4    (+3750 lines) */
    0x1F0E, /* 12: 1/2    (+7950 lines) */
    0x3FDE, /* 13: 1s     (+16350 lines) */
    0x817E, /* 14: 2s     (+33150 lines) */
    0x0000  /* 15: AUTO   */
};

static const char *const s_shutter_label_tbl[OV7725_SHUTTER_COUNT] = {
    "1/8000", "1/4000", "1/2000", "1/1000",
    "1/500",  "1/300",  "1/150",  "1/100",
    "1/50",   "1/25",   "1/8",    "1/4",
    "1/2",    "1s",     "2s",     "AUTO"
};

/* OV7725 GAIN(0x00) 档位: Gain = (b7+1)*(b6+1)*(b5+1)*(b4+1)*(1+b[3:0]/16) */
static const uint8_t s_iso_gain_tbl[OV7725_ISO_COUNT] = {
    0x00, /* 0: AUTO (AGC on) */
    0x00, /* 1: ISO 100  (1x)  */
    0x10, /* 2: ISO 200  (2x)  */
    0x30, /* 3: ISO 400  (4x)  */
    0x70, /* 4: ISO 800  (8x)  */
    0xF0  /* 5: ISO 1600 (16x) */
};

static const char *const s_iso_label_tbl[OV7725_ISO_COUNT] = {
    "A", "100", "200", "400", "800", "1600"
};

const char *ov7725_shutter_label(uint8_t dial_pos)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 0;
    return s_shutter_label_tbl[dial_pos];
}

const char *ov7725_iso_label(uint8_t iso_idx)
{
    if (iso_idx >= OV7725_ISO_COUNT) iso_idx = 0;
    return s_iso_label_tbl[iso_idx];
}

/**
 * @brief  切换预览模式: 0 = 全视野 QVGA 320x240; 1 = VGA 原生中心 160x120 (4X面积放大对焦)
 */
void ov7725_set_preview_mode(uint8_t zoom_4x)
{
    if (!zoom_4x)
    {
        ov7725_window_set(320, 240);
    }
    else
    {
        /* VGA RGB565 (0x06), 无降采样, 取 640x480 正中心 160x120 窗口 */
        ov7725_write_reg(COM7,     0x06);   /* VGA RGB565 */
        ov7725_write_reg(HSTART,   0x5F);   /* 水平起始 240 */
        ov7725_write_reg(HSIZE,    0x28);   /* 宽度 160 (0x28 << 2) */
        ov7725_write_reg(VSTRT,    0x61);   /* 垂直起始 180 */
        ov7725_write_reg(VSIZE,    0x3C);   /* 高度 120 (0x3C << 1) */
        ov7725_write_reg(HREF,     0x00);
        ov7725_write_reg(HOutSize, 0x28);   /* 输出宽度 160 */
        ov7725_write_reg(VOutSize, 0x3C);   /* 输出高度 120 */
        ov7725_write_reg(EXHCH,    0x00);
    }
}

void ov7725_apply_exposure(uint8_t dial_pos, uint8_t iso_idx, uint8_t is_raw_mode)
{
    uint8_t com8 = is_raw_mode ? 0x02U : 0xFAU;

    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 0;
    if (iso_idx  >= OV7725_ISO_COUNT)     iso_idx  = 0;

    if (OV7725_DIAL_IS_AUTO(dial_pos))
    {
        com8 |= 0x01U; /* 使能 AEC */
    }
    else
    {
        com8 &= (uint8_t)(~0x01U); /* 关闭 AEC，手动曝光 */
    }

    if (iso_idx == 0)
    {
        com8 |= 0x04U; /* 使能 AGC */
    }
    else
    {
        com8 &= (uint8_t)(~0x04U); /* 关闭 AGC，手动 ISO */
    }

    ov7725_write_reg(COM8, com8);

    if (!OV7725_DIAL_IS_AUTO(dial_pos))
    {
        uint16_t lines = s_shutter_aec_tbl[dial_pos];
        uint16_t dummy = s_shutter_dummy_tbl[dial_pos];
        ov7725_write_reg(COM5,   0x00U); /* 关闭自动降帧，使用精确手动哑行 */
        ov7725_write_reg(DM_LNH, (uint8_t)((dummy >> 8) & 0xFFU));
        ov7725_write_reg(DM_LNL, (uint8_t)(dummy & 0xFFU));
        ov7725_write_reg(AECH,   (uint8_t)((lines >> 8) & 0xFFU));
        ov7725_write_reg(AEC,    (uint8_t)(lines & 0xFFU));
    }
    else
    {
        ov7725_write_reg(DM_LNH, 0x00U);
        ov7725_write_reg(DM_LNL, 0x00U);
        ov7725_write_reg(COM5,   is_raw_mode ? 0x00U : 0xF5U);
    }

    if (iso_idx > 0)
    {
        ov7725_write_reg(GAIN, s_iso_gain_tbl[iso_idx]);
    }
}

void ov7725_snapshot_auto_exposure(uint16_t *aec_lines, uint8_t *gain_reg)
{
    uint8_t aech = ov7725_read_reg(AECH);
    uint8_t aec  = ov7725_read_reg(AEC);
    if (aec_lines)
    {
        uint16_t v = (uint16_t)(((uint16_t)aech << 8) | aec);
        *aec_lines = (v == 0) ? 0x0060U : v;
    }
    if (gain_reg)
    {
        *gain_reg = ov7725_read_reg(GAIN);
    }
}

uint8_t ov7725_is_slow_shutter(uint8_t dial_pos)
{
    return (dial_pos >= 10U && dial_pos <= 14U) ? 1U : 0U;
}

uint8_t ov7725_is_auto_shutter(uint8_t dial_pos)
{
    return OV7725_DIAL_IS_AUTO(dial_pos) ? 1U : 0U;
}

uint8_t ov7725_iso_to_gain_reg(uint8_t iso_idx)
{
    if (iso_idx >= OV7725_ISO_COUNT) iso_idx = 1U;
    return s_iso_gain_tbl[iso_idx];
}

/* Convert target linear gain in Q4 fixed-point (16 = 1.0x .. 496 = 31.0x) to OV7725 GAIN(0x00) */
uint8_t ov7725_gain_q4_to_reg(uint16_t gain_q4)
{
    uint8_t hi_bits = 0;
    uint16_t mult = 1;

    if (gain_q4 <= 16U) return 0x00U;
    if (gain_q4 >= 496U) return 0xFFU;

    while ((uint16_t)(mult * 32U) <= gain_q4 && hi_bits < 4U)
    {
        mult <<= 1;
        hi_bits++;
    }
    /* Remaining fractional part: gain_q4 / mult in [16..31] -> low nibble = frac - 16 */
    uint16_t rem = (uint16_t)((gain_q4 + (mult >> 1)) / mult);
    uint8_t lo = (rem > 16U) ? (uint8_t)(rem - 16U) : 0U;
    if (lo > 15U) lo = 15U;

    uint8_t hi_mask = (uint8_t)(((1U << hi_bits) - 1U) << 4);
    return (uint8_t)(hi_mask | lo);
}

/* Decode OV7725 GAIN(0x00) register back into Q4 linear gain (16 = 1.0x) */
static uint16_t ov7725_gain_reg_to_q4(uint8_t reg)
{
    uint16_t mult = 1U;
    if (reg & 0x10U) mult <<= 1;
    if (reg & 0x20U) mult <<= 1;
    if (reg & 0x40U) mult <<= 1;
    if (reg & 0x80U) mult <<= 1;
    return (uint16_t)(mult * (uint16_t)(16U + (reg & 0x0FU)));
}

/*
 * Always-RAW live viewfinder exposure setter:
 *   - For fast shutters (0..9: 1/8000..1/25) and AUTO (15), applies EXACT capture
 *     AEC/DM_LN/GAIN so shutter lag is 0..33ms and exposure simulation is 100% physical.
 *   - For slow shutters (10..14: 1/8s..2s), caps preview exposure at OV7725_PREVIEW_MAX_AEC
 *     (~1/12s, keeping viewfinder >=12fps smooth) and compensates brightness via temporary
 *     hardware GAIN + returned digital LUT boost (Q8: 256 = 1.0x .. 1024 = 4.0x).
 */
uint16_t ov7725_apply_always_raw_preview(uint8_t dial_pos, uint8_t iso_idx, uint16_t auto_aec, uint8_t auto_gain)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 15U;
    if (iso_idx  >= OV7725_ISO_COUNT)     iso_idx  = 0U;

    uint16_t target_aec = OV7725_DIAL_IS_AUTO(dial_pos) ? auto_aec : s_shutter_aec_tbl[dial_pos];
    if (target_aec < 4U) target_aec = 4U;

    uint8_t  base_gain_reg = (iso_idx == 0U) ? auto_gain : s_iso_gain_tbl[iso_idx];
    uint16_t prev_aec      = target_aec;
    uint16_t prev_dummy    = 0U;
    uint8_t  prev_gain_reg = base_gain_reg;
    uint16_t digital_q8    = 256U; /* 1.0x */

    if (prev_aec > OV7725_PREVIEW_MAX_AEC)
    {
        /* Slow shutter live-view boost: cap exposure at ~1/12s for >=12fps preview */
        prev_aec   = OV7725_PREVIEW_MAX_AEC;
        prev_dummy = (uint16_t)(prev_aec - 450U); /* 510 + 950 = 1460 > 1400 lines */

        uint32_t base_q4   = ov7725_gain_reg_to_q4(base_gain_reg);
        uint32_t needed_q4 = (base_q4 * (uint32_t)target_aec) / (uint32_t)prev_aec;
        if (needed_q4 > 496U)
        {
            prev_gain_reg = 0xFFU; /* Max 31x hardware gain */
            uint32_t dig = (needed_q4 * 256UL) / 496UL;
            if (dig > 1024UL) dig = 1024UL; /* Cap digital boost at 4.0x */
            digital_q8 = (uint16_t)dig;
        }
        else
        {
            prev_gain_reg = ov7725_gain_q4_to_reg((uint16_t)needed_q4);
        }
    }
    else if (prev_aec > 490U)
    {
        prev_dummy = (uint16_t)(prev_aec - 450U);
    }

    ov7725_write_reg(COM8,   0x02U);
    ov7725_write_reg(COM5,   0x00U);
    ov7725_write_reg(DM_LNH, (uint8_t)((prev_dummy >> 8) & 0xFFU));
    ov7725_write_reg(DM_LNL, (uint8_t)(prev_dummy & 0xFFU));
    ov7725_write_reg(AECH,   (uint8_t)((prev_aec >> 8) & 0xFFU));
    ov7725_write_reg(AEC,    (uint8_t)(prev_aec & 0xFFU));
    ov7725_write_reg(GAIN,   prev_gain_reg);

    return digital_q8;
}

void ov7725_format_shutter_label(uint8_t dial_pos, uint16_t auto_aec, char *buf, uint8_t buf_len)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 15U;
    if (!OV7725_DIAL_IS_AUTO(dial_pos))
    {
        snprintf(buf, buf_len, "%s", s_shutter_label_tbl[dial_pos]);
        return;
    }
    /* Approximate shutter speed from 16,800 lines/sec reference (336 lines = 1/50s) */
    if (auto_aec < 4U) auto_aec = 4U;
    uint32_t denom = (16800UL + (auto_aec / 2U)) / auto_aec;
    if (denom >= 1000UL)
        snprintf(buf, buf_len, "A1/%u", (unsigned int)((denom / 100UL) * 100UL));
    else if (denom >= 100UL)
        snprintf(buf, buf_len, "A1/%u", (unsigned int)((denom / 10UL) * 10UL));
    else if (denom >= 2UL)
        snprintf(buf, buf_len, "A1/%u", (unsigned int)denom);
    else
        snprintf(buf, buf_len, "A 1s");
}

void ov7725_apply_raw_exposure(uint8_t dial_pos, uint8_t iso_idx, uint16_t auto_aec, uint8_t auto_gain)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 15U;
    if (iso_idx  >= OV7725_ISO_COUNT)     iso_idx  = 0U;

    /* RAW 拍照阶段锁定 AE/AG，防止过渡帧中亮度跳变 */
    ov7725_write_reg(COM8, 0x02U);
    ov7725_write_reg(COM5, 0x00U);

    uint16_t lines = OV7725_DIAL_IS_AUTO(dial_pos) ? auto_aec : s_shutter_aec_tbl[dial_pos];
    if (lines < 4U) lines = 4U;
    uint16_t dummy = 0U;
    if (!OV7725_DIAL_IS_AUTO(dial_pos))
    {
        dummy = s_shutter_dummy_tbl[dial_pos];
    }
    else if (lines > 490U)
    {
        dummy = (uint16_t)(lines - 450U);
    }
    uint8_t gain = (iso_idx == 0U) ? auto_gain : s_iso_gain_tbl[iso_idx];

    ov7725_write_reg(DM_LNH, (uint8_t)((dummy >> 8) & 0xFFU));
    ov7725_write_reg(DM_LNL, (uint8_t)(dummy & 0xFFU));
    ov7725_write_reg(AECH,   (uint8_t)((lines >> 8) & 0xFFU));
    ov7725_write_reg(AEC,    (uint8_t)(lines & 0xFFU));
    ov7725_write_reg(GAIN,   gain);
}











