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


/* Shadow cache so live AE never rewrites unchanged CLKRC/COM8/COM5 mid-frame */
static uint8_t  s_raw_cache_valid = 0U;
static uint8_t  s_raw_last_clkrc  = 0xFFU;
static uint16_t s_raw_last_dummy  = 0xFFFFU;
static uint16_t s_raw_last_aec    = 0xFFFFU;
static uint8_t  s_raw_last_gain   = 0xFFU;

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
    s_raw_cache_valid = 0U;
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

/* 16 档快门拨盘映射表 (COM4=0x81 6x PLL, 12MHz XCLK, 18MHz PCLK):
 *   0..9  = 快门与 1/25 档: CLKRC=0x00 (22,959 行/秒, 45fps 基准), 精确物理曝光时间
 *   10..14= 慢门手动档 (1/8..2s): 拍照瞬间切 CLKRC=0x01 (11,480 行/秒, 2s=22959 行)
 *   15    = AUTO 自动测光档 (CLKRC=0x00, 1..459 行 = 1/23000..1/50s, 全程 DM_LN=0)
 */
#define OV7725_DIAL_IS_AUTO(pos)  ((pos) >= 15U)

static const uint16_t s_shutter_aec_tbl[OV7725_SHUTTER_COUNT] = {
    0x0003, /* 0:  1/8000 (3 lines   @22.96kHz = 1/7653s) */
    0x0006, /* 1:  1/4000 (6 lines   = 1/3826s) */
    0x000B, /* 2:  1/2000 (11 lines  = 1/2087s) */
    0x0017, /* 3:  1/1000 (23 lines  = 1/998s)  */
    0x002E, /* 4:  1/500  (46 lines  = 1/499s)  */
    0x004D, /* 5:  1/300  (77 lines  = 1/298s)  */
    0x0099, /* 6:  1/150  (153 lines = 1/150s)  */
    0x00E6, /* 7:  1/100  (230 lines = 1/100s)  */
    0x01CB, /* 8:  1/50   (459 lines = 1/50s, fits in 45fps DM_LN=0!) */
    0x0396, /* 9:  1/25   (918 lines = 1/25s)   */
    0x059B, /* 10: 1/8    (1435 lines  @11.48kHz CLKRC=0x01) */
    0x0B36, /* 11: 1/4    (2870 lines  @11.48kHz CLKRC=0x01) */
    0x166C, /* 12: 1/2    (5740 lines  @11.48kHz CLKRC=0x01) */
    0x2CD8, /* 13: 1s     (11480 lines @11.48kHz CLKRC=0x01) */
    0x59AF, /* 14: 2s     (22959 lines @11.48kHz CLKRC=0x01) */
    0x0000  /* 15: AUTO   */
};

/* 附加哑行数 (DM_LNH/DM_LNL: 保证总帧行数 510 + dummy > AEC) */
static const uint16_t s_shutter_dummy_tbl[OV7725_SHUTTER_COUNT] = {
    0x0000, /* 0:  1/8000 */
    0x0000, /* 1:  1/4000 */
    0x0000, /* 2:  1/2000 */
    0x0000, /* 3:  1/1000 */
    0x0000, /* 4:  1/500  */
    0x0000, /* 5:  1/300  */
    0x0000, /* 6:  1/150  */
    0x0000, /* 7:  1/100  */
    0x0000, /* 8:  1/50   (459 < 510 -> DM_LN=0, full 45fps!) */
    0x01CA, /* 9:  1/25   (+458 lines -> 968 total, 23.7fps) */
    0x03D9, /* 10: 1/8    (+985 lines @ CLKRC=0x01) */
    0x0974, /* 11: 1/4    (+2420 lines @ CLKRC=0x01) */
    0x14AA, /* 12: 1/2    (+5290 lines @ CLKRC=0x01) */
    0x2B16, /* 13: 1s     (+11030 lines @ CLKRC=0x01) */
    0x57ED, /* 14: 2s     (+22509 lines @ CLKRC=0x01) */
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
 * Always-RAW live viewfinder exposure setter (CLKRC=0x00, 60fps base, 15.7ms rolling shutter):
 *   - For fast shutters (0..9: 1/8000..1/25) and AUTO (15), applies EXACT capture
 *     AEC/DM_LN/GAIN so shutter lag is 0..17ms and exposure simulation is 100% physical.
 *   - For slow shutters (10..14: 1/8s..2s), caps preview exposure at OV7725_PREVIEW_MAX_AEC
 *     (~1/30s = 33ms sensor period, keeping viewfinder >=15fps smooth) and compensates
 *     brightness via temporary hardware GAIN + returned digital LUT boost (Q8: 256..1024).
 */
uint16_t ov7725_apply_always_raw_preview(uint8_t dial_pos, uint8_t iso_idx, uint16_t auto_aec, uint8_t auto_gain)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 15U;
    if (iso_idx  >= OV7725_ISO_COUNT)     iso_idx  = 0U;

    uint8_t  base_gain_reg = (iso_idx == 0U) ? auto_gain : s_iso_gain_tbl[iso_idx];
    uint16_t prev_aec;
    uint16_t prev_dummy    = 0U;
    uint8_t  prev_gain_reg = base_gain_reg;
    uint16_t digital_q8    = 256U; /* 1.0x */

    if (ov7725_is_slow_shutter(dial_pos))
    {
        /* Slow shutter live-view boost: keep sensor at full 60fps (DM_LN=0, 500 lines -> 16F!) */
        uint32_t target_aec_fast = (uint32_t)s_shutter_aec_tbl[dial_pos] * 2UL;
        prev_aec   = OV7725_PREVIEW_SLOW_AEC;
        prev_dummy = 0U;

        uint32_t base_q4   = ov7725_gain_reg_to_q4(base_gain_reg);
        uint32_t needed_q4 = (base_q4 * target_aec_fast) / (uint32_t)prev_aec;
        if (needed_q4 > 496U)
        {
            prev_gain_reg = 0xFFU; /* Max 31x hardware gain */
            uint32_t dig = (needed_q4 * 256UL) / 496UL;
            if (dig > 1024UL) dig = 1024UL; /* Cap digital boost at 4.0x (124x total covers 2s) */
            digital_q8 = (uint16_t)dig;
        }
        else
        {
            prev_gain_reg = ov7725_gain_q4_to_reg((uint16_t)needed_q4);
        }
    }
    else if (OV7725_DIAL_IS_AUTO(dial_pos))
    {
        prev_aec = auto_aec;
        if (prev_aec < OV7725_AUTO_MIN_AEC) prev_aec = OV7725_AUTO_MIN_AEC;
        if (prev_aec > OV7725_AUTO_MAX_AEC) prev_aec = OV7725_AUTO_MAX_AEC;
        if (prev_aec > 490U)
        {
            prev_dummy = (uint16_t)(prev_aec - 460U);
        }
    }
    else
    {
        /* Fast shutters 0..9: exact capture lines & dummy at CLKRC=0x00 */
        prev_aec   = s_shutter_aec_tbl[dial_pos];
        prev_dummy = s_shutter_dummy_tbl[dial_pos];
    }

    /* Only write registers that actually changed; NEVER touch CLKRC unless clock mode changed! */
    if (!s_raw_cache_valid)
    {
        ov7725_write_reg(COM8, 0x02U);
        ov7725_write_reg(COM5, 0x00U);
    }
    if (!s_raw_cache_valid || s_raw_last_clkrc != 0x00U)
    {
        ov7725_write_reg(CLKRC, 0x00U);
        s_raw_last_clkrc = 0x00U;
    }
    if (!s_raw_cache_valid || s_raw_last_dummy != prev_dummy)
    {
        ov7725_write_reg(DM_LNH, (uint8_t)((prev_dummy >> 8) & 0xFFU));
        ov7725_write_reg(DM_LNL, (uint8_t)(prev_dummy & 0xFFU));
        s_raw_last_dummy = prev_dummy;
    }
    if (!s_raw_cache_valid || s_raw_last_aec != prev_aec)
    {
        ov7725_write_reg(AECH, (uint8_t)((prev_aec >> 8) & 0xFFU));
        ov7725_write_reg(AEC,  (uint8_t)(prev_aec & 0xFFU));
        s_raw_last_aec = prev_aec;
    }
    if (!s_raw_cache_valid || s_raw_last_gain != prev_gain_reg)
    {
        ov7725_write_reg(GAIN, prev_gain_reg);
        s_raw_last_gain = prev_gain_reg;
    }
    s_raw_cache_valid = 1U;

    return digital_q8;
}

void ov7725_format_shutter_value(uint8_t dial_pos, uint16_t auto_aec, char *buf, uint8_t buf_len)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 15U;
    if (!OV7725_DIAL_IS_AUTO(dial_pos))
    {
        snprintf(buf, buf_len, "%s", s_shutter_label_tbl[dial_pos]);
        return;
    }
    /* Exact shutter speed at COM4=0x81, CLKRC=0x00 (22,959 lines/sec) */
    if (auto_aec < 1U) auto_aec = 1U;
    if (auto_aec == 1U)
    {
        snprintf(buf, buf_len, "1/23000");
        return;
    }
    if (auto_aec == 2U)
    {
        snprintf(buf, buf_len, "1/11500");
        return;
    }
    uint32_t denom = (22959UL + (auto_aec / 2U)) / auto_aec;
    if (denom >= 1000UL)
        snprintf(buf, buf_len, "1/%u", (unsigned int)(((denom + 50UL) / 100UL) * 100UL));
    else if (denom >= 100UL)
        snprintf(buf, buf_len, "1/%u", (unsigned int)(((denom + 5UL) / 10UL) * 10UL));
    else if (denom >= 2UL)
        snprintf(buf, buf_len, "1/%u", (unsigned int)denom);
    else
        snprintf(buf, buf_len, "1s");
}

/* Format ISO value (1/3-stop clean steps in AUTO mode: 100, 125, 160, 200, ... 1600) */
void ov7725_format_iso_value(uint8_t iso_idx, uint16_t auto_gain_q4, char *buf, uint8_t buf_len)
{
    if (iso_idx > 0U && iso_idx < OV7725_ISO_COUNT)
    {
        snprintf(buf, buf_len, "%s", s_iso_label_tbl[iso_idx]);
        return;
    }
    static const uint16_t s_iso_steps[13] = {
        100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600
    };
    uint32_t iso = ((uint32_t)auto_gain_q4 * 100UL + 8UL) / 16UL;
    uint16_t best = s_iso_steps[0];
    uint32_t best_diff = 10000UL;
    for (uint8_t i = 0; i < 13U; i++)
    {
        uint32_t d = (iso > s_iso_steps[i]) ? (iso - s_iso_steps[i]) : (s_iso_steps[i] - iso);
        if (d < best_diff)
        {
            best_diff = d;
            best = s_iso_steps[i];
        }
    }
    snprintf(buf, buf_len, "%u", (unsigned int)best);
}

void ov7725_apply_raw_exposure(uint8_t dial_pos, uint8_t iso_idx, uint16_t auto_aec, uint8_t auto_gain)
{
    if (dial_pos >= OV7725_SHUTTER_COUNT) dial_pos = 15U;
    if (iso_idx  >= OV7725_ISO_COUNT)     iso_idx  = 0U;

    /* Slow shutters (10..14) drop CLKRC to 0x01 (15.3kHz lines) so 2s fits 16-bit AEC/DM_LN */
    uint8_t clkrc = ov7725_is_slow_shutter(dial_pos) ? 0x01U : 0x00U;
    ov7725_write_reg(CLKRC, clkrc);
    ov7725_write_reg(COM8,  0x02U);
    ov7725_write_reg(COM5,  0x00U);

    uint16_t lines = OV7725_DIAL_IS_AUTO(dial_pos) ? auto_aec : s_shutter_aec_tbl[dial_pos];
    if (lines < 1U) lines = 1U;
    uint16_t dummy = 0U;
    if (!OV7725_DIAL_IS_AUTO(dial_pos))
    {
        dummy = s_shutter_dummy_tbl[dial_pos];
    }
    else if (lines > 490U)
    {
        dummy = (uint16_t)(lines - 460U);
    }
    uint8_t gain = (iso_idx == 0U) ? auto_gain : s_iso_gain_tbl[iso_idx];

    ov7725_write_reg(DM_LNH, (uint8_t)((dummy >> 8) & 0xFFU));
    ov7725_write_reg(DM_LNL, (uint8_t)(dummy & 0xFFU));
    ov7725_write_reg(AECH,   (uint8_t)((lines >> 8) & 0xFFU));
    ov7725_write_reg(AEC,    (uint8_t)(lines & 0xFFU));
    ov7725_write_reg(GAIN,   gain);

    s_raw_last_clkrc  = clkrc;
    s_raw_last_dummy  = dummy;
    s_raw_last_aec    = lines;
    s_raw_last_gain   = gain;
    s_raw_cache_valid = 1U;
}











