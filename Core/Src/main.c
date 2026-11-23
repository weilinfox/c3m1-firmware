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
#include "../../BSP/TPAD/tpad.h"
#include "../../BSP/OV7725/ov7725.h"
#include "../../SYSTEM/delay/delay.h"
#include "../../BSP/STMFLASH/stmflash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t g_vsync_state;   /* 在exit.c里 面定义 */
extern uint8_t g_ov7725_frame; /* 在timer.c里面定义 */
//volatile uint16_t IMGNUM=1;
volatile uint8_t RAW_shot=0; /*定义FIFO里的是第几次填满的图片，第十张才会被输出*/
//const uint16_t g_save_buf = 0; /* 要写入到FLASH保存的快门次数 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_SAVE_ADDR 0X08070000 /* 设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小 + 0X08000000) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


void ov7725_camera_refresh(void) /*更新LCD显示，该函数将OV7725模块FIFO里面的数据拷贝到LCD屏幕上*/
{
    uint16_t color;//定义了一个16位的变量color存储一个像素的数据。

    if (g_vsync_state==2) /* 标志位为2时，说明FIFO当前已经完整写入新的一帧了*/
    {
    	/*以下是LCD的内容*/
        lcd_scan_dir(U2D_L2R); /* LCD从上到下, 从左到右 */
        lcd_set_window(0, 0, 320, 240); /* 将LCD显示区域设置到屏幕中央 */
        lcd_write_ram_prepare(); /* 开始写入LCD GRAM */

        /*以下是摄像头内容*/
        OV7725_WEN(0);  // 双重禁写，保险中的保险
		OV7725_OE(0);
		__NOP();        // 等待信号稳定

        OV7725_RRST(0); /* 复位读指针到开头 */
        OV7725_RCLK(0); /* 先关掉时钟输入保证是低电平 */
        OV7725_RCLK(1); /* 打开时钟输入 */
        OV7725_RCLK(0); /* 再关掉时钟输入，这三步制造了一个上升沿*/
        OV7725_RRST(1); /* 复位读指针结束 */
        OV7725_RCLK(1); /* 开始读时钟输入 */

        register uint16_t i = 240; // 寄存器变量，访问更快
        while (i--) // 递减循环，判断条件更简单（i != 0）
        {
            register uint16_t j = 320;
            while (j--)
            {
            	OV7725_RCLK(0);
				color = OV7725_DATA; /* 读数据 */
				OV7725_RCLK(1);
				color <<= 8;
				OV7725_RCLK(0);
				color |= OV7725_DATA; /* 读数据 */
				OV7725_RCLK(1);
				/*把像素数据写入LCD的GRAM*/
				LCD->LCD_RAM = color;
            }
        }

        g_vsync_state = 0; /* 清零帧中断标记 */
        lcd_scan_dir(DFT_SCAN_DIR); /* 恢复默认扫描方向 */
//        g_ov7725_frame++;  /*统计帧数然后打印出来*/
    }
}

//void USART1_SendByte(uint8_t data) {
//    while (!(USART1->SR & USART_SR_TXE)); // 等待TXE位（发送寄存器空）置位
//    USART1->DR = data;                    // 写入数据
//}

const uint8_t dng_header[] = {
0x49, 0x49, 0x2A, 0x00, 0x08, 0x00, 0x00, 0x00, 0x1D, 0x00, 0xFE, 0x00, 0x04, 0x00, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x84, 0x02,
0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x02, 0x01,
0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x01, 0x03, 0x00, 0x01, 0x00,
0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x23, 0x80,
0x00, 0x00, 0x11, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x02, 0x00, 0x00, 0x12, 0x01,
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x16, 0x01, 0x04, 0x00, 0x01, 0x00,
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
0x48, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01,
0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
0x46, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0xCE, 0xFE, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00,
0x6E, 0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00, 0x46, 0xFE, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00,
0x6C, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
0x98, 0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
0x58, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xE0, 0x01, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

FRESULT res;    //FATFS的返回值，用来判断是否成功的
FATFS fs;       //FATFS有关的定义
FIL file;       //FATFS有关的定义
UINT bw;		//FATFS有关的定义
volatile uint16_t ShutterCount;

void __attribute__((optimize("O0")))ov7725_camera_output(void)/*这个是用来输出10bitRAW数据流的程序*//*__attribute__((optimize("O0")))加上这个的意思是禁止优化*/
{
	/*FLASH_SAVE_ADDR：Flash 读取的起始地址；SIZE：读取的 半字（uint16_t）数量，总字节数 = SIZE * 2*/
	/*(uint16_t *)datatemp 就是告诉 stmflash_read 函数：“请把从 Flash 里读出来的数据，以 16 位为单位，存到 datatemp 这个 RAM 缓冲区里。”*/
	stmflash_read(FLASH_SAVE_ADDR, (uint16_t *)&ShutterCount, 1); /*先读flash里面存的快门次数数据*/
	ShutterCount++;/*把快门次数+1*/
	char filename[16]; /*定义一个16个字符的文件名数组，最多可以写入99999这样的文件名*/
	sprintf(filename, "0:CDP_%d.dng", ShutterCount); /*将 “固定格式” 与 “动态变量” 拼接成完整字符串，存入指定缓冲区（这里是 filename）*/
	f_open(&file,filename,FA_CREATE_ALWAYS | FA_WRITE); /*创建DNG文件*/
	f_write(&file, dng_header, sizeof(dng_header), &bw);   /*写入DNG头*/

	stmflash_write(FLASH_SAVE_ADDR, (uint16_t *)&ShutterCount, 1);

    uint16_t i, j;
    uint8_t Pixel,low_byte,high_byte;
    uint16_t Pixel2;

    printf("SDWriting!\r\n");

    // 1. 定义4kb缓冲区（匹配FATFS扇区大小，块写效率最高）
    static uint8_t buf[4096];/*设定4KB的缓冲空间，全局变量防止栈溢出*/
	uint16_t buf_idx = 0;  // 缓冲区当前填充索引

	while(1)
	{
		if (g_vsync_state==2)  /* 标志位为2时，说明上一帧已经完全写入FIFO */
		{
			LED1(0);
			OV7725_WEN(0);          // 双重禁写，保险中的保险
			OV7725_OE(0);
			__NOP();        // 等待信号稳定

			OV7725_RRST(0); /* 复位读指针到开头 */
			OV7725_RCLK(0); /* 先关掉时钟输入保证是低电平 */
			OV7725_RCLK(1); /* 打开时钟输入 */
			OV7725_RCLK(0); /* 再关掉时钟输入，这三步制造了一个上升沿*/
			OV7725_RRST(1); /* 复位读指针结束 */
			OV7725_RCLK(1); /* 开始读时钟输入 */

			for (i = 0; i < 480; i++)
			{
				for (j = 0; j < 644; j++)
				{
					OV7725_RCLK(0);
					Pixel = OV7725_DATA; /* 读数据 */
					OV7725_RCLK(1);
					Pixel2 = Pixel<<8; /*还原成10bit RAW但其实低2位还是00*/

					low_byte  =  Pixel2 & 0xFF;       /*低字节*//* 用printf %c直接发送原始字节（无格式转换），小端序*/
					high_byte = (Pixel2 >> 8) & 0xFF; /*高字节*/
//					USART1_SendByte(low_byte);
//					USART1_SendByte(high_byte);

					// 2. 数据存入缓冲区（不再直接写SD卡）
					buf[buf_idx++] = low_byte;
					buf[buf_idx++] = high_byte;

					// 3. 缓冲区满512字节，立即块写入SD卡（核心优化）
					if (buf_idx >= 4096)
					{
						f_write(&file, buf, 4096, &bw);  // 一次写512字节（块写）
						buf_idx = 0;  // 重置索引，准备下次填充
					}
				}
			}

			if (buf_idx > 0)
			{
				f_write(&file, buf, buf_idx, &bw);  // 写剩余字节
				buf_idx = 0;
			}

			g_vsync_state = 0; /* 清零帧中断标记 */
			break;
		}

	}
	f_close(&file);  /*关闭已经写入的文件*/
//	IMGNUM++;        /*文件名+1*/
	LED1(1);
	printf("SD_Write_DONE!\r\n");
}

void __attribute__((optimize("O0")))Shot(void)
{
	while(1)
			  {
				  if(g_vsync_state==2)  //如果来了一个FIFO满信号
				  {
					  g_vsync_state=0; //就把这个信号清空，让它再写入新的信号
					  RAW_shot++;       //然后来的次数计数+1
					  if(RAW_shot==5)    //如果计数达到了6
					  {
						  g_vsync_state=0;
						  RAW_shot=0;
						  break;        //就跳出循环，去拍摄照片
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
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  res=f_mount(&fs,"0:",1);  /*挂载SD卡*/
  if(res==FR_OK){
	printf("SD_OK!\r\n");
  }
  else{
	printf("SD_ERR:%d\r\n",res);
  }

  lcd_init();   /* 初始化LCD */
  delay_ms(5);
  ov7725_init();/*如果摄像头初始化返回值是0也就是成功*//*此时OV7725已经按照初始SCCB参数，在往FIFO写入数据了，但是尚未读取*/
  ov7725_window_set(320, 240);/* QVGA模式输出320*240,输出QVGA实时显示只是把SCCB的内容稍微改了一下 */

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
