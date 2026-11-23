/* USER CODE BEGIN Header */
/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       OV7725摄像头 实验
 * 实验平台:正点原子 STM32F103开发板
 ****************************************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../BSP/LED/led.h"
#include "../../BSP/KEY/key.h"
#include "../../BSP/LCD/lcd.h"
#include "../../BSP/OV7725/ov7725.h"
#include "../../BSP/EXTI/exti.h"
#include "../../SYSTEM/delay/delay.h"
#include "../../BSP/STMFLASH/stmflash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t g_ov7725_frame; /* 在tim.c里面定义 */
volatile uint8_t RAW_shot = 0; /* 定义FIFO里丢弃的过渡帧计数 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_SAVE_ADDR 0X08070000 /* 设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小 + 0X08000000) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


void ov7725_camera_refresh(void) /* 更新LCD显示: AL422B FIFO (PG1~PG8) -> 8位 FSMC ST7789V3 */
{
    if (g_vsync_state == 2) /* 标志位为2时，说明FIFO当前已经完整写入新的一帧了 */
    {
        /* 横屏 320x240 全屏开窗 */
        lcd_set_window(0, 0, 320, 240);
        lcd_write_ram_prepare();

        /* 禁止继续写入并开启 FIFO 输出 */
        OV7725_WEN(0);
        OV7725_OE(0);
        __NOP();

        OV7725_RRST(0); /* 复位读指针到开头 */
        OV7725_RCLK(0);
        OV7725_RCLK(1);
        OV7725_RCLK(0);
        OV7725_RRST(1); /* 复位读指针结束 */
        OV7725_RCLK(1);

        /* OV7725 RGB565 输出为高字节先、低字节后；ST7789V3 8位并口 16bpp 亦为高字节先、低字节后，直接流式写入 */
        uint32_t pixels = 320UL * 240UL;
        while (pixels--)
        {
            OV7725_RCLK(0);
            LCD->LCD_RAM = OV7725_DATA; /* RGB565 高字节 */
            OV7725_RCLK(1);
            OV7725_RCLK(0);
            LCD->LCD_RAM = OV7725_DATA; /* RGB565 低字节 */
            OV7725_RCLK(1);
        }

        g_vsync_state = 0; /* 清零帧中断标记，允许下一帧写入 FIFO */
    }
}

//void USART1_SendByte(uint8_t data) {
//    while (!(USART1->SR & USART_SR_TXE)); // 等待TXE位（发送寄存器空）置位
//    USART1->DR = data;                    // 写入数据
//}

/* 592字节 DNG 1.4 (TIFF/EP) 文件头：644x480 BGGR 16-bit 容器，有效裁切 640x480
 * 已对齐生产固件修复项：
 *   - 0x033: BitsPerSample count = 1 (原误写为 257)
 *   - 0x066: Orientation = 3 (180度旋转，匹配机身内倒装 CMOS)
 *   - 0x177: X/YResolution 分母 = 1 (72/1 DPI)
 *   - 0x1A0..0x1D1: 校准后的 ColorMatrix1
 *   - 0x217: NoiseProfile 第6个 double 指数修正为 0x3E
 */
const uint8_t dng_header[] = {
0x49, 0x49, 0x2A, 0x00, 0x08, 0x00, 0x00, 0x00, 0x1D, 0x00, 0xFE, 0x00, 0x04, 0x00, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x84, 0x02,
0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x02, 0x01,
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x01, 0x03, 0x00, 0x01, 0x00,
0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x23, 0x80,
0x00, 0x00, 0x11, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x02, 0x00, 0x00, 0x12, 0x01,
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x16, 0x01, 0x04, 0x00, 0x01, 0x00,
0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x17, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x6F,
0x09, 0x00, 0x1A, 0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x70, 0x01, 0x00, 0x00, 0x1B, 0x01,
0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x70, 0x01, 0x00, 0x00, 0x28, 0x01, 0x03, 0x00, 0x01, 0x00,
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x8D, 0x82, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00,
0x02, 0x00, 0x8E, 0x82, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x02, 0x01, 0x01, 0x00, 0x16, 0x92,
0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x12, 0xC6, 0x01, 0x00, 0x04, 0x00,
0x00, 0x00, 0x01, 0x04, 0x00, 0x00, 0x16, 0xC6, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01,
0x02, 0x00, 0x17, 0xC6, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x19, 0xC6,
0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x1A, 0xC6, 0x03, 0x00, 0x04, 0x00,
0x00, 0x00, 0x78, 0x01, 0x00, 0x00, 0x1D, 0xC6, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF,
0x00, 0x00, 0x1E, 0xC6, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x1F, 0xC6,
0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xC6, 0x03, 0x00, 0x02, 0x00,
0x00, 0x00, 0x80, 0x02, 0xE0, 0x01, 0x21, 0xC6, 0x0A, 0x00, 0x09, 0x00, 0x00, 0x00, 0x90, 0x01,
0x00, 0x00, 0x8D, 0xC6, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0xD8, 0x01, 0x00, 0x00, 0x41, 0xC7,
0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0xC7, 0x0C, 0x00, 0x06, 0x00,
0x00, 0x00, 0xE8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x48, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01,
0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
0x46, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0xCE, 0xFE, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00,
0x92, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x46, 0xFE, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00,
0x6C, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x34, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
0x98, 0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
0x58, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xE0, 0x01, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

FRESULT res;
FATFS fs;
FIL file;
UINT bw;
volatile uint16_t ShutterCount = 0;
uint8_t g_sd_ready = 0;

static uint16_t shutter_count_load(void)
{
	uint16_t val = 0;
	stmflash_read(FLASH_SAVE_ADDR, &val, 1);
	if (val == 0xFFFFU)
	{
		val = 0;
	}
	ShutterCount = val;
	return val;
}

static void shutter_count_reset(void)
{
	uint16_t zero = 0;
	stmflash_write(FLASH_SAVE_ADDR, &zero, 1);
	ShutterCount = 0;
	printf("ShutterCount reset to 0\r\n");
}

static uint8_t sd_try_mount(void)
{
	if (!sd_card_is_present())
	{
		g_sd_ready = 0;
		printf("NO_SD_CARD\r\n");
		return 0;
	}
	if (MX_SDIO_SD_Init() != 0)
	{
		g_sd_ready = 0;
		printf("SDIO init failed\r\n");
		return 0;
	}
	res = f_mount(&fs, "0:", 1);
	if (res == FR_OK)
	{
		g_sd_ready = 1;
		printf("SD_Card_Mounted_Successfully\r\n");
		return 1;
	}
	g_sd_ready = 0;
	printf("SD_ERR:%d\r\n", res);
	return 0;
}

uint8_t ov7725_camera_output(void) /* 输出 RAW DNG 文件，返回 0 表示成功 */
{
	uint16_t count_tmp = 0;
	FRESULT fres;

	stmflash_read(FLASH_SAVE_ADDR, &count_tmp, 1); /* 先读flash里面存的快门次数数据 */
	count_tmp++; /* 快门次数+1 (首次0xFFFF自然回绕为0) */
	ShutterCount = count_tmp;

	/* 8.3 短文件名兼容整个 uint16_t 范围: CDI00000.dng ~ CDI65535.dng */
	char filename[20];
	snprintf(filename, sizeof(filename), "0:CDI%05u.dng", (unsigned int)count_tmp);

	fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (fres != FR_OK)
	{
		printf("SD open err:%d (%s)\r\n", fres, filename);
		return 1;
	}

	fres = f_write(&file, dng_header, sizeof(dng_header), &bw);
	if (fres != FR_OK || bw != sizeof(dng_header))
	{
		printf("SD hdr write err:%d\r\n", fres);
		f_close(&file);
		return 2;
	}

	stmflash_write(FLASH_SAVE_ADDR, &count_tmp, 1);

	uint16_t i, j;
	uint8_t Pixel;

	printf("SDWriting: %s\r\n", filename);

	/* 4KB 缓冲区（8 个 SD 扇区，块写效率最高） */
	static uint8_t buf[4096];
	uint16_t buf_idx = 0;

	while (1)
	{
		if (g_vsync_state == 2)  /* 标志位为2时，说明上一帧已经完全写入FIFO */
		{
			LED1(0);
			OV7725_WEN(0);
			OV7725_OE(0);
			__NOP();

			OV7725_RRST(0);
			OV7725_RCLK(0);
			OV7725_RCLK(1);
			OV7725_RCLK(0);
			OV7725_RRST(1);
			OV7725_RCLK(1);

			for (i = 0; i < 480; i++)
			{
				for (j = 0; j < 644; j++)
				{
					OV7725_RCLK(0);
					Pixel = OV7725_DATA; /* 读高8位RAW数据 */
					OV7725_RCLK(1);

					/* 小端 16-bit 容器，有效 8-bit 放高字节 (Pixel << 8) */
					buf[buf_idx++] = 0x00;
					buf[buf_idx++] = Pixel;

					if (buf_idx >= sizeof(buf))
					{
						fres = f_write(&file, buf, sizeof(buf), &bw);
						buf_idx = 0;
						if (fres != FR_OK || bw != sizeof(buf))
						{
							printf("SD write err:%d\r\n", fres);
							g_vsync_state = 0;
							f_close(&file);
							LED1(1);
							return 3;
						}
					}
				}
			}

			if (buf_idx > 0)
			{
				fres = f_write(&file, buf, buf_idx, &bw);
				buf_idx = 0;
				if (fres != FR_OK)
				{
					printf("SD tail write err:%d\r\n", fres);
					g_vsync_state = 0;
					f_close(&file);
					LED1(1);
					return 3;
				}
			}

			g_vsync_state = 0;
			break;
		}
	}

	fres = f_close(&file);
	LED1(1);
	if (fres != FR_OK)
	{
		printf("SD close err:%d\r\n", fres);
		return 4;
	}
	printf("SD_Write_DONE!\r\n");
	return 0;
}

void Shot(void)
{
	RAW_shot = 0;
	while (1)
	{
		if (g_vsync_state == 2)
		{
			g_vsync_state = 0;
			RAW_shot++;
			if (RAW_shot >= 5)
			{
				RAW_shot = 0;
				break;
			}
		}
	}
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	volatile uint8_t key;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  delay_init(72);						/* 初始化延时函数 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();   /* 初始化8位 ST7789V3 LCD */
  delay_ms(5);

  /* 读取当前快门计数；若开机时按住 NUMRST (PA3) 则归零 */
  if (HAL_GPIO_ReadPin(NUMRST_GPIO_Port, NUMRST_Pin) == GPIO_PIN_RESET)
  {
    shutter_count_reset();
    lcd_show_string(60, 110, 200, 16, 16, "COUNTER RESET -> 0", YELLOW);
    delay_ms(600);
    lcd_clear(BLACK);
  }
  else
  {
    shutter_count_load();
  }

  printf("BAT: %umV (%u%%) NUM:%u\r\n",
         adc_battery_get_mv(), adc_battery_get_percent(), (unsigned int)ShutterCount);

  if (!sd_try_mount())
  {
    lcd_show_string(70, 96, 200, 16, 16, "NO SD CARD!", RED);
    lcd_show_string(36, 120, 260, 16, 16, "Insert card to save DNG", WHITE);
    delay_ms(1200);
    lcd_clear(BLACK);
  }

  ov7725_init();
  ov7725_window_set(320, 240);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) /*下面这个循环其实就是在一直刷新画面，然后等是否有按键按下*/
  {

	  ov7725_camera_refresh(); /* 更新显示 */
	  key = key_scan(0); /* 不支持连按 */

	  if(key)
	  {
		  ov7725_RAW_init(); /*重新按RAW格式初始化*/
		  g_vsync_state=0;  /*清零标志位*/
//		  while(1)
//		  {
//			  if(g_vsync_state==2)  //如果来了一个FIFO满信号
//			  {
//				  g_vsync_state=0; //就把这个信号清空，让它再写入新的信号
//				  RAW_shot++;       //然后来的次数计数+1
//				  if(RAW_shot==5)    //如果计数达到了6
//				  {
//					  g_vsync_state=0;
//					  RAW_shot=0;
//					  break;        //就跳出循环，去拍摄照片
//				  }
//			  }
//		  }
		  Shot();
		  ov7725_camera_output();
		  ov7725_init();
		  ov7725_window_set(320, 240);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  /*出错干啥都写这边*/
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
