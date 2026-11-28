/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdio.c
  * @brief   SDIO 4-bit + DMA2_Channel4 driver (HWFC disabled per STM32F103 ES0340).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "sdio.h"
#include "bsp_driver_sd.h"
#include "c3m1_log.h"
#include <string.h>

/* USER CODE BEGIN 0 */
uint8_t g_sd_sta = 0;

/* Target & fallback dividers (SDIOCLK = 72 MHz, SDIO_CK = 72 / (ClockDiv + 2)) */
#define C3M1_SDIO_CLKDIV_24M       0x01U /* 24.0 MHz (4-bit DMA2, <= 25 MHz SD Default Speed) */
#define C3M1_SDIO_CLKDIV_18M       0x02U /* 18.0 MHz fallback tier 1 */
#define C3M1_SDIO_CLKDIV_12M       0x04U /* 12.0 MHz fallback tier 2 */
#define C3M1_SDIO_CLKDIV_FALLBACK  0x06U /*  9.0 MHz conservative floor */
#define C3M1_SDIO_DMA_MAX_BLOCKS   64U   /* 32 KB = 8192 words per DMA2 burst (<= 65535 CNDTR) */

SD_HandleTypeDef  hsd;
DMA_HandleTypeDef hdma_sdio;

static uint32_t          s_sd_dma_bounce[128]; /* 512 B 4-byte aligned bounce buffer for unaligned callers */
static volatile uint8_t  s_sd_xfer_done    = 0U; /* 0=busy, 1=rx_ok, 2=tx_ok, 3=err/abort */
static volatile uint32_t s_sd_last_err     = 0UL;
static uint32_t          s_sd_active_clkdiv = C3M1_SDIO_CLKDIV_FALLBACK;

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

static void sdio_abort_clean(void)
{
  (void)HAL_SD_Abort(&hsd);
  if (hdma_sdio.State != HAL_DMA_STATE_READY)
  {
    (void)HAL_DMA_Abort(&hdma_sdio);
  }
  __HAL_SD_DISABLE_IT(&hsd, SDIO_IT_DATAEND  | SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT |
                            SDIO_IT_TXUNDERR | SDIO_IT_RXOVERR  | SDIO_IT_STBITERR);
  hsd.Instance->DCTRL = 0U;
  __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_FLAGS | SDIO_FLAG_STBITERR);
  hsd.State   = HAL_SD_STATE_READY;
  hsd.Context = SD_CONTEXT_NONE;
}

static HAL_StatusTypeDef sdio_wait_card_transfer(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
  {
    if (!sd_card_is_present())
    {
      s_sd_last_err = 0x40000000UL;
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - t0) >= timeout_ms)
    {
      s_sd_last_err = 0x20000000UL | hsd.ErrorCode;
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/* Aligned multi-block DMA2 transfer chunk (1..C3M1_SDIO_DMA_MAX_BLOCKS sectors) */
static HAL_StatusTypeDef sdio_dma_xfer_chunk(uint8_t is_read, uint8_t *aligned_buf,
                                             uint32_t sector, uint32_t blocks,
                                             uint32_t timeout_ms)
{
  if (!sd_card_is_present())
  {
    s_sd_last_err = 0x40000000UL;
    return HAL_ERROR;
  }

  if (hsd.State != HAL_SD_STATE_READY || hdma_sdio.State != HAL_DMA_STATE_READY)
  {
    sdio_abort_clean();
  }

  if (sdio_wait_card_transfer(timeout_ms) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_sd_xfer_done = 0U;
  s_sd_last_err  = 0UL;
  /* STM32F1 HAL bug: HAL_DMA_IRQHandler leaves DMA_CCR_EN=1 after TC, which
   * write-protects DMA_CCR_DIR when HAL_SD_{Read,Write}Blocks_DMA switches RX<->TX
   * on the shared half-duplex DMA2_Channel4! Disable channel before HAL touches DIR. */
  __HAL_DMA_DISABLE(&hdma_sdio);
  __DMB();

  HAL_StatusTypeDef st = is_read
      ? HAL_SD_ReadBlocks_DMA(&hsd, aligned_buf, sector, blocks)
      : HAL_SD_WriteBlocks_DMA(&hsd, aligned_buf, sector, blocks);

  if (st != HAL_OK)
  {
    s_sd_last_err = 0x10000000UL | hsd.ErrorCode;
    sdio_abort_clean();
    return HAL_ERROR;
  }

  uint32_t t0 = HAL_GetTick();
  while (s_sd_xfer_done == 0U)
  {
    if (!sd_card_is_present())
    {
      s_sd_last_err = 0x40000000UL;
      sdio_abort_clean();
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - t0) >= timeout_ms)
    {
      s_sd_last_err = 0x80000000UL | hsd.ErrorCode | (hsd.Instance->STA & 0x3FU);
      sdio_abort_clean();
      return HAL_TIMEOUT;
    }
  }

  uint8_t expect = is_read ? 1U : 2U;
  if (s_sd_xfer_done != expect || hsd.ErrorCode != HAL_SD_ERROR_NONE)
  {
    s_sd_last_err |= hsd.ErrorCode;
    if (s_sd_last_err == 0UL)
    {
      s_sd_last_err = 0x08000000UL;
    }
    sdio_abort_clean();
    return HAL_ERROR;
  }

  if (is_read)
  {
    /* Guard against HAL RX race: DMA TC can fire ~16 SDIO_CK before final-block CRC check finishes */
    uint32_t t_end = HAL_GetTick();
    while ((hsd.Instance->STA & SDIO_FLAG_RXACT) != 0U)
    {
      if ((HAL_GetTick() - t_end) >= 10U)
      {
        break;
      }
    }
    uint32_t late_sta = hsd.Instance->STA &
                        (SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_RXOVERR |
                         SDIO_FLAG_STBITERR | SDIO_FLAG_RXACT);
    if (late_sta != 0U || s_sd_xfer_done != 1U || hsd.ErrorCode != HAL_SD_ERROR_NONE)
    {
      s_sd_last_err |= hsd.ErrorCode | late_sta | 0x04000000UL;
      sdio_abort_clean();
      return HAL_ERROR;
    }
    __HAL_SD_DISABLE_IT(&hsd, SDIO_IT_DATAEND  | SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT |
                              SDIO_IT_TXUNDERR | SDIO_IT_RXOVERR  | SDIO_IT_STBITERR);
    hsd.Instance->DCTRL &= ~(SDIO_DCTRL_DTEN | SDIO_DCTRL_DMAEN);
    __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_DATA_FLAGS | SDIO_FLAG_STBITERR);
  }
  else
  {
    uint32_t tx_err = hsd.Instance->STA &
                      (SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_TXUNDERR | SDIO_FLAG_STBITERR);
    if (tx_err != 0U || s_sd_xfer_done != 2U || hsd.ErrorCode != HAL_SD_ERROR_NONE)
    {
      s_sd_last_err |= hsd.ErrorCode | tx_err | 0x04000000UL;
      sdio_abort_clean();
      return HAL_ERROR;
    }
    /* HAL_SD_IRQHandler leaves Context & static data flags set on DMA TX completion */
    __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STATIC_DATA_FLAGS | SDIO_FLAG_STBITERR);
    hsd.Context = SD_CONTEXT_NONE;
  }

  __HAL_DMA_DISABLE(&hdma_sdio);
  return HAL_OK;
}

static HAL_StatusTypeDef sdio_dma_read_sync(uint8_t *buf, uint32_t sector, uint32_t count, uint32_t timeout_ms)
{
  if (buf == NULL || count == 0U)
  {
    return HAL_ERROR;
  }

  if ((((uintptr_t)buf) & 3U) == 0U)
  {
    while (count > 0U)
    {
      uint32_t chunk = (count > C3M1_SDIO_DMA_MAX_BLOCKS) ? C3M1_SDIO_DMA_MAX_BLOCKS : count;
      HAL_StatusTypeDef st = sdio_dma_xfer_chunk(1U, buf, sector, chunk, timeout_ms);
      if (st != HAL_OK)
      {
        return st;
      }
      buf    += chunk * 512U;
      sector += chunk;
      count  -= chunk;
    }
    return HAL_OK;
  }

  /* Unaligned caller buffer: bounce 1 sector at a time through 4-byte aligned SRAM */
  while (count > 0U)
  {
    HAL_StatusTypeDef st = sdio_dma_xfer_chunk(1U, (uint8_t *)s_sd_dma_bounce, sector, 1U, timeout_ms);
    if (st != HAL_OK)
    {
      return st;
    }
    memcpy(buf, s_sd_dma_bounce, 512U);
    buf    += 512U;
    sector += 1U;
    count  -= 1U;
  }
  return HAL_OK;
}

static HAL_StatusTypeDef sdio_dma_write_sync(const uint8_t *buf, uint32_t sector, uint32_t count, uint32_t timeout_ms)
{
  if (buf == NULL || count == 0U)
  {
    return HAL_ERROR;
  }

  if ((((uintptr_t)buf) & 3U) == 0U)
  {
    while (count > 0U)
    {
      uint32_t chunk = (count > C3M1_SDIO_DMA_MAX_BLOCKS) ? C3M1_SDIO_DMA_MAX_BLOCKS : count;
      HAL_StatusTypeDef st = sdio_dma_xfer_chunk(0U, (uint8_t *)buf, sector, chunk, timeout_ms);
      if (st != HAL_OK)
      {
        return st;
      }
      buf    += chunk * 512U;
      sector += chunk;
      count  -= chunk;
    }
    return HAL_OK;
  }

  /* Unaligned caller buffer: copy into 4-byte aligned bounce sector before DMA2 TX */
  while (count > 0U)
  {
    memcpy(s_sd_dma_bounce, buf, 512U);
    HAL_StatusTypeDef st = sdio_dma_xfer_chunk(0U, (uint8_t *)s_sd_dma_bounce, sector, 1U, timeout_ms);
    if (st != HAL_OK)
    {
      return st;
    }
    buf    += 512U;
    sector += 1U;
    count  -= 1U;
  }
  return HAL_OK;
}

static uint8_t sdio_try_init_div(uint32_t clk_div, uint8_t verify_read)
{
  s_sd_last_err = 0UL;
  if (hsd.Instance == SDIO && hsd.State != HAL_SD_STATE_RESET)
  {
    HAL_SD_DeInit(&hsd);
  }

  /* 1-bit init -> 4-bit wide bus; HWFC MUST stay DISABLED on STM32F103 (Errata ES0340 s2.8.1) */
  hsd.Instance                 = SDIO;
  hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_ENABLE;
  hsd.Init.BusWide             = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv            = clk_div;

  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    s_sd_last_err = hsd.ErrorCode;
    HAL_SD_DeInit(&hsd);
    return 2U;
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    s_sd_last_err = hsd.ErrorCode;
    HAL_SD_DeInit(&hsd);
    return 3U;
  }

  if (verify_read)
  {
    /* Multi-block (2-sector CMD18 + CMD12) DMA2 read twice and verify FNV-1a match */
    uint32_t probe_buf[256];
    if (sdio_dma_read_sync((uint8_t *)probe_buf, 0U, 2U, 300U) != HAL_OK)
    {
      HAL_SD_DeInit(&hsd);
      return 4U;
    }
    uint32_t h1 = sdio_buf_checksum(probe_buf, 256U);
    if (sdio_dma_read_sync((uint8_t *)probe_buf, 0U, 2U, 300U) != HAL_OK ||
        sdio_buf_checksum(probe_buf, 256U) != h1)
    {
      HAL_SD_DeInit(&hsd);
      return 4U;
    }
  }

  s_sd_active_clkdiv = clk_div;
  return 0U;
}
/* USER CODE END 0 */

/* SDIO init function */

uint8_t sd_card_is_present(void)
{
  return (HAL_GPIO_ReadPin(SDOK_GPIO_Port, SDOK_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

uint32_t sdio_get_active_clkdiv(void)
{
  return s_sd_active_clkdiv;
}

uint32_t sdio_get_last_error(void)
{
  return s_sd_last_err;
}

uint8_t sdio_init_with_div(uint32_t clk_div)
{
  if (!sd_card_is_present())
  {
    if (hsd.Instance == SDIO && hsd.State != HAL_SD_STATE_RESET)
    {
      HAL_SD_DeInit(&hsd);
    }
    g_sd_sta = 1;
    return 1U;
  }

  uint8_t rc = sdio_try_init_div(clk_div, 0U);
  g_sd_sta = rc;
  return rc;
}

void MX_SDIO_SD_DeInit(void)
{
  if (hsd.Instance == SDIO && hsd.State != HAL_SD_STATE_RESET)
  {
    HAL_SD_DeInit(&hsd);
  }
  g_sd_sta = 1;
}

void sdio_irq_handler(void)
{
  uint32_t sta = hsd.Instance->STA;
  uint32_t err_flags = sta & (SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT |
                              SDIO_FLAG_RXOVERR  | SDIO_FLAG_TXUNDERR | SDIO_FLAG_STBITERR);
  if (err_flags != 0U)
  {
    s_sd_last_err |= err_flags;
    if ((err_flags & (SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT |
                      SDIO_FLAG_RXOVERR  | SDIO_FLAG_TXUNDERR)) != 0U)
    {
      /* Clear DATAEND first so HAL_SD_IRQHandler takes its error branch instead of swallowing final-block errors */
      __HAL_SD_CLEAR_FLAG(&hsd, SDIO_FLAG_DATAEND);
    }
    else
    {
      /* STBITERR alone is unhandled by HAL_SD_IRQHandler; signal error cleanly */
      hsd.ErrorCode |= HAL_SD_ERROR_DATA_CRC_FAIL;
      __HAL_SD_CLEAR_FLAG(&hsd, SDIO_FLAG_STBITERR | SDIO_FLAG_DATAEND);
      __HAL_SD_DISABLE_IT(&hsd, SDIO_IT_DATAEND  | SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT |
                                SDIO_IT_TXUNDERR | SDIO_IT_RXOVERR  | SDIO_IT_STBITERR);
      s_sd_xfer_done = 3U;
      return;
    }
  }
  HAL_SD_IRQHandler(&hsd);
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

  /* Auto-probe 4-bit DMA2 speed ladder: 24 MHz -> 18 MHz -> 12 MHz -> 9 MHz conservative floor */
  static const uint32_t s_fast_divs[] = {
    C3M1_SDIO_CLKDIV_24M,
    C3M1_SDIO_CLKDIV_18M,
    C3M1_SDIO_CLKDIV_12M
  };

  for (uint8_t i = 0U; i < (uint8_t)(sizeof(s_fast_divs) / sizeof(s_fast_divs[0])); i++)
  {
    uint32_t div = s_fast_divs[i];
    uint8_t rc = sdio_try_init_div(div, 1U);
    if (rc == 0U)
    {
      C3M1_LOG("SDIO: 4-bit DMA2 div=%lu OK\r\n", (unsigned long)div);
      g_sd_sta = 0;
      return 0;
    }
    C3M1_LOG("SDIO: div=%lu probe err=%u code=0x%08lX\r\n",
             (unsigned long)div, (unsigned int)rc, (unsigned long)s_sd_last_err);
  }

  uint8_t rc = sdio_try_init_div(C3M1_SDIO_CLKDIV_FALLBACK, 0U);
  g_sd_sta = rc;
  return rc;
}

void HAL_SD_MspInit(SD_HandleTypeDef *sdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (sdHandle->Instance == SDIO)
  {
    /* Peripheral & DMA2 clocks */
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10    ------> SDIO_D2
    PC11    ------> SDIO_D3
    PC12    ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    GPIO_InitStruct.Pin   = SD_D0_Pin | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | SD_CLK_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = SD_CMD_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CMD_GPIO_Port, &GPIO_InitStruct);

    /* Single half-duplex DMA2_Channel4 shared by SDIO RX & TX (HAL sets Direction per xfer) */
    hdma_sdio.Instance                 = DMA2_Channel4;
    hdma_sdio.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sdio.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sdio.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sdio.Init.Mode                = DMA_NORMAL;
    hdma_sdio.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    (void)HAL_DMA_DeInit(&hdma_sdio);
    if (HAL_DMA_Init(&hdma_sdio) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(sdHandle, hdmarx, hdma_sdio);
    __HAL_LINKDMA(sdHandle, hdmatx, hdma_sdio);

    /* Same preemption priority (1) prevents SDIO_IRQn from preempting DMA2's CMD12 wait,
     * while subpriority (0 < 1) still services SDIO first when both are pending simultaneously. */
    HAL_NVIC_SetPriority(SDIO_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *sdHandle)
{
  if (sdHandle->Instance == SDIO)
  {
    HAL_NVIC_DisableIRQ(DMA2_Channel4_5_IRQn);
    HAL_NVIC_DisableIRQ(SDIO_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Channel4_5_IRQn);
    HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);

    (void)HAL_DMA_DeInit(&hdma_sdio);
    sdHandle->hdmarx = NULL;
    sdHandle->hdmatx = NULL;

    __HAL_RCC_DMA2_CLK_DISABLE();
    __HAL_RCC_SDIO_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOC, SD_D0_Pin | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | SD_CLK_Pin);
    HAL_GPIO_DeInit(SD_CMD_GPIO_Port, SD_CMD_Pin);
  }
}

/* ======================== HAL / BSP DMA Callbacks ======================== */

void BSP_SD_ReadCpltCallback(void)
{
  if (s_sd_xfer_done == 0U && hsd.ErrorCode == HAL_SD_ERROR_NONE)
  {
    s_sd_xfer_done = 1U;
  }
  else
  {
    s_sd_last_err |= hsd.ErrorCode;
    s_sd_xfer_done = 3U;
  }
}

void BSP_SD_WriteCpltCallback(void)
{
  if (s_sd_xfer_done == 0U && hsd.ErrorCode == HAL_SD_ERROR_NONE)
  {
    s_sd_xfer_done = 2U;
  }
  else
  {
    s_sd_last_err |= hsd.ErrorCode;
    s_sd_xfer_done = 3U;
  }
}

void BSP_SD_AbortCallback(void)
{
  s_sd_last_err |= hsd.ErrorCode;
  s_sd_xfer_done = 3U;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd_cb)
{
  s_sd_last_err |= hsd_cb->ErrorCode;
  s_sd_xfer_done = 3U;
}

/* Override weak polling BSP hooks so FatFs (sd_diskio.c) automatically uses DMA2 */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  return (sdio_dma_read_sync((uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) == HAL_OK)
         ? MSD_OK : MSD_ERROR;
}

uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  return (sdio_dma_write_sync((const uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) == HAL_OK)
         ? MSD_OK : MSD_ERROR;
}

/* USER CODE BEGIN 1 */

uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo)
{
  return (uint8_t)HAL_SD_GetCardInfo(&hsd, cardinfo);
}

uint8_t get_sd_card_state(void)
{
  return ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
  if (sdio_dma_read_sync(pbuf, saddr, cnt, SD_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return (sdio_wait_card_transfer(SD_TIMEOUT) == HAL_OK) ? HAL_OK : SD_TRANSFER_BUSY;
}

uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
  if (sdio_dma_write_sync(pbuf, saddr, cnt, SD_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return (sdio_wait_card_transfer(SD_TIMEOUT) == HAL_OK) ? HAL_OK : SD_TRANSFER_BUSY;
}

/* USER CODE END 1 */
