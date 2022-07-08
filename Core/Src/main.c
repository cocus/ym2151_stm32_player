/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include <ctype.h>
#include "vgmheader.h"
#include "megastream.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, -1);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED0_Pin|YM_ICL_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUS_D0_Pin|BUS_D1_Pin|BUS_D2_Pin|BUS_D3_Pin
                          |BUS_D4_Pin|BUS_D5_Pin|BUS_D6_Pin|BUS_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YM_A0_Pin|YM_WR_Pin|YM_CS_Pin|SPI_CS_Pin
                          |YM_RD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED0_Pin YM_ICL_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|YM_ICL_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUS_D0_Pin BUS_D1_Pin BUS_D2_Pin BUS_D3_Pin
                           BUS_D4_Pin BUS_D5_Pin BUS_D6_Pin BUS_D7_Pin */
  GPIO_InitStruct.Pin = BUS_D0_Pin|BUS_D1_Pin|BUS_D2_Pin|BUS_D3_Pin
                          |BUS_D4_Pin|BUS_D5_Pin|BUS_D6_Pin|BUS_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : YM_A0_Pin YM_WR_Pin YM_CS_Pin SPI_CS_Pin
                           YM_RD_Pin */
  GPIO_InitStruct.Pin = YM_A0_Pin|YM_WR_Pin|YM_CS_Pin|SPI_CS_Pin
                          |YM_RD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_DET_Pin USB_DET_Pin */
  GPIO_InitStruct.Pin = SD_DET_Pin|USB_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN4_Pin BTN3_Pin BTN2_Pin BTN1_Pin
                           BTN0_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin|BTN3_Pin|BTN2_Pin|BTN1_Pin
                          |BTN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

volatile int32_t waitSamples = 0;

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim4)
  {
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
   if (waitSamples)
   {
    waitSamples--;
   }
  }
}


#define HAL_GPIO_WriteNamedPin(name, value) HAL_GPIO_WritePin(name##_GPIO_Port, name##_Pin, value)

/* Code taken from https://github.com/AidanHockey5/YM2151_Arcade_Classic_2/ */

void ym2151_write(uint8_t addr, uint8_t data)
{
    HAL_GPIO_WriteNamedPin(YM_CS, 0); // CS Low
    HAL_GPIO_WriteNamedPin(YM_A0, 0); // A0 Low

    HAL_GPIO_WriteNamedPin(BUS_D0, addr & (1 << 0));
    HAL_GPIO_WriteNamedPin(BUS_D1, addr & (1 << 1));
    HAL_GPIO_WriteNamedPin(BUS_D2, addr & (1 << 2));
    HAL_GPIO_WriteNamedPin(BUS_D3, addr & (1 << 3));
    HAL_GPIO_WriteNamedPin(BUS_D4, addr & (1 << 4));
    HAL_GPIO_WriteNamedPin(BUS_D5, addr & (1 << 5));
    HAL_GPIO_WriteNamedPin(BUS_D6, addr & (1 << 6));
    HAL_GPIO_WriteNamedPin(BUS_D7, addr & (1 << 7));

    __NOP(); // 240ns

    HAL_GPIO_WriteNamedPin(YM_WR, 0); // WR Low
    __NOP(); // 240ns
    HAL_GPIO_WriteNamedPin(YM_WR, 1); // WR High
    __NOP(); // 125ns
    HAL_GPIO_WriteNamedPin(YM_A0, 1); // A0 High
    __NOP(); // 125ns

    HAL_GPIO_WriteNamedPin(BUS_D0, data & (1 << 0));
    HAL_GPIO_WriteNamedPin(BUS_D1, data & (1 << 1));
    HAL_GPIO_WriteNamedPin(BUS_D2, data & (1 << 2));
    HAL_GPIO_WriteNamedPin(BUS_D3, data & (1 << 3));
    HAL_GPIO_WriteNamedPin(BUS_D4, data & (1 << 4));
    HAL_GPIO_WriteNamedPin(BUS_D5, data & (1 << 5));
    HAL_GPIO_WriteNamedPin(BUS_D6, data & (1 << 6));
    HAL_GPIO_WriteNamedPin(BUS_D7, data & (1 << 7));

    __NOP(); // 240ns

    HAL_GPIO_WriteNamedPin(YM_WR, 0); // WR Low
    __NOP(); // 240ns
    HAL_GPIO_WriteNamedPin(YM_WR, 1); // WR High

    HAL_GPIO_WriteNamedPin(YM_CS, 1); // CS High

    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP(); //Busy flag on chip will be set, but we can't read it since MCU is 3.3V. 11 micros is about the time where the worst-case busy clear time is satisfied.
             //Writing again too quickly will corrupt the data on the chip

}

void ym2151_reset(void)
{
    HAL_GPIO_WriteNamedPin(YM_ICL, 0); // ICL Low
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP(); // 25us
    HAL_GPIO_WriteNamedPin(YM_ICL, 1); // ICL High

    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP(); // 25us
}


void samples_wait(uint32_t samples)
{
    waitSamples = samples;
    while(waitSamples > 0) { };
}

//MegaStreamContext_t stream;
//uint8_t buf[VGM_BUF_SIZE];

uint32_t loopPos = 0; //The location of the 0x66 command
uint16_t loopCount = 0;
//volatile int32_t waitSamples = 0;
uint16_t badCommandCount = 0;

void process_vgm(char* path)
{
    vgm_header header;

    FIL fil; 		//File handle
    FRESULT fres; //Result after operations

    myprintf("%s: processing '%s'\n", __func__, path);

    fres = f_open(&fil, path, FA_READ);
    if (fres != FR_OK)
    {
        myprintf("f_open error (%i)\n", fres);
        return;
    }

    unsigned int read = 0;
    fres = f_read(&fil, &header, sizeof(header), &read);
    if (fres != FR_OK)
    {
        myprintf("f_read error (%i)\n", fres);
        goto cleanup;
    }
    if (read != sizeof(header))
    {
        myprintf("f_read size mismatch (%i)\n", fres);
        goto cleanup;
    }

    myprintf("  indent = 0x%x\n", header.indent);
    myprintf("  EoF = 0x%x\n", header.EoF);
    myprintf("  version = 0x%x\n", header.version);
    myprintf("  sn76489Clock = 0x%x\n", header.sn76489Clock);
    myprintf("  ym2413Clock = 0x%x\n", header.ym2413Clock);
    myprintf("  gd3Offset = 0x%x\n", header.gd3Offset);
    myprintf("  totalSamples = 0x%x\n", header.totalSamples);
    myprintf("  loopOffset = 0x%x\n", header.loopOffset);
    myprintf("  loopNumSamples = 0x%x\n", header.loopNumSamples);
    myprintf("  rate = 0x%x\n", header.rate);
    myprintf("  snX = 0x%x\n", header.snX);
    myprintf("  ym2612Clock = 0x%x\n", header.ym2612Clock);
    myprintf("  ym2151Clock = 0x%x\n", header.ym2151Clock);
    myprintf("  vgmDataOffset = 0x%x\n", header.vgmDataOffset);
    myprintf("  segaPCMClock = 0x%x\n", header.segaPCMClock);
    myprintf("  spcmInterface = 0x%x\n", header.spcmInterface);
    myprintf("  rf5C68clock = 0x%x\n", header.rf5C68clock);
    myprintf("  ym2203clock = 0x%x\n", header.ym2203clock);
    myprintf("  ym2608clock = 0x%x\n", header.ym2608clock);
    myprintf("  ym2610clock = 0x%x\n", header.ym2610clock);
    myprintf("  ym3812clock = 0x%x\n", header.ym3812clock);
    myprintf("  ym3526clock = 0x%x\n", header.ym3526clock);
    myprintf("  y8950clock = 0x%x\n", header.y8950clock);
    myprintf("  ymf262clock = 0x%x\n", header.ymf262clock);
    myprintf("  ymf278bclock = 0x%x\n", header.ymf278bclock);
    myprintf("  ymf271clock = 0x%x\n", header.ymf271clock);
    myprintf("  ymz280Bclock = 0x%x\n", header.ymz280Bclock);
    myprintf("  rf5C164clock = 0x%x\n", header.rf5C164clock);
    myprintf("  pwmclock = 0x%x\n", header.pwmclock);
    myprintf("  ay8910clock = 0x%x\n", header.ay8910clock);
    myprintf("  ayclockflags = 0x%x\n", header.ayclockflags);
    myprintf("  vmlblm = 0x%x\n", header.vmlblm);
    myprintf("  gbdgmclock = 0x%x\n", header.gbdgmclock);
    myprintf("  nesapuclock = 0x%x\n", header.nesapuclock);
    myprintf("  multipcmclock = 0x%x\n", header.multipcmclock);
    myprintf("  upd7759clock = 0x%x\n", header.upd7759clock);
    myprintf("  okim6258clock = 0x%x\n", header.okim6258clock);
    myprintf("  ofkfcf = 0x%x\n", header.ofkfcf);
    myprintf("  okim6295clock = 0x%x\n", header.okim6295clock);
    myprintf("  k051649clock = 0x%x\n", header.k051649clock);
    myprintf("  k054539clock = 0x%x\n", header.k054539clock);
    myprintf("  huc6280clock = 0x%x\n", header.huc6280clock);
    myprintf("  c140clock = 0x%x\n", header.c140clock);
    myprintf("  k053260clock = 0x%x\n", header.k053260clock);
    myprintf("  pokeyclock = 0x%x\n", header.pokeyclock);
    myprintf("  qsoundclock = 0x%x\n", header.qsoundclock);
    myprintf("  scspclock = 0x%x\n", header.scspclock);
    myprintf("  extrahdrofs = 0x%x\n", header.extrahdrofs);
    myprintf("  wonderswanclock = 0x%x\n", header.wonderswanclock);
    myprintf("  vsuClock = 0x%x\n", header.vsuClock);
    myprintf("  saa1099clock = 0x%x\n", header.saa1099clock);
    myprintf("  es5503clock = 0x%x\n", header.es5503clock);
    myprintf("  es5506clock = 0x%x\n", header.es5506clock);
    myprintf("  eschcdxx = 0x%x\n", header.eschcdxx);
    myprintf("  x1010clock = 0x%x\n", header.x1010clock);
    myprintf("  c352clock = 0x%x\n", header.c352clock);
    myprintf("  ga20clock = 0x%x\n", header.ga20clock);

    if (header.vgmDataOffset == 0)
    {
        f_lseek(&fil, 0x40);
    }
    else
    {
        f_lseek(&fil, header.vgmDataOffset + 0x34);
    }

    if(header.gd3Offset != 0)
    {
        loopPos = header.gd3Offset+0x14-1;
    }
    else
    {
        loopPos = header.EoF+4-1;
    }

    ym2151_reset();


    uint8_t breakit = 0;
    while(breakit == 0)
    {
        uint8_t cmd;
        uint8_t data;
        uint8_t addr;
        uint16_t nsamples;
        fres = f_read(&fil, &cmd, 1, &read);
        if (read != 1)
        {
            myprintf("read != 1 (%i)\r\n", read);
            goto cleanup;
        }

        switch (cmd)
        {
            case 0x31:
            {
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x4f:
            {
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x50:
            {
                f_read(&fil, &data, 1, &read);
                break;
            }

            case 0x51:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x52:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x53:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x54:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);

               // myprintf("ym2151_write addr = 0x%x, data = 0x%x\r\n", addr, data);
                ym2151_write(addr, data);
                break;
            }
            case 0x55:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x56:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x57:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x58:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x59:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x5a:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x5b:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x5c:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x5d:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x5e:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }
            case 0x5f:
            {
                f_read(&fil, &addr, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }

            case 0x61:
            {
                f_read(&fil, &nsamples, 2, &read);
                samples_wait(nsamples);
                break;
            }
            case 0x62:
            {
                samples_wait(735);
                break;
            }
            case 0x63:
            {
                samples_wait(882);
                break;
            }
            case 0x66:
            {
                //Loop
                //if(maxLoops != 0xFFFF) //If sent to short int max, just loop forever
                //    loopCount++;
                breakit = 1;
                break;
            }
            case 0x67:
            {
                f_read(&fil, &data, 1, &read);
                f_read(&fil, &data, 1, &read);
                f_read(&fil, &data, 1, &read);
                uint32_t pcmSize;
                f_read(&fil, &pcmSize, 4, &read);
                for(uint32_t i=0; i<pcmSize; i++)
                {
                    f_read(&fil, &data, 1, &read);
                }
            }
            case 0x68:
            {
                myprintf("Not handled 0x68\r\n");
                break;
            }


            case 0x70:
            case 0x71:
            case 0x72:
            case 0x73:
            case 0x74:
            case 0x75:
            case 0x76:
            case 0x77:
            case 0x78:
            case 0x79:
            case 0x7A:
            case 0x7B:
            case 0x7C:
            case 0x7D:
            case 0x7E:
            case 0x7F:
            {
                samples_wait((cmd & 0xf) + 1);
                break;
            }

            case 0x80:
            case 0x81:
            case 0x82:
            case 0x83:
            case 0x84:
            case 0x85:
            case 0x86:
            case 0x87:
            case 0x88:
            case 0x89:
            case 0x8A:
            case 0x8B:
            case 0x8C:
            case 0x8D:
            case 0x8E:
            case 0x8F:
            {
                break;
            }


            case 0x90:
            {
                f_read(&fil, &nsamples, 2, &read);
                f_read(&fil, &nsamples, 2, &read);
                break;
            }
            case 0x91:
            {
                f_read(&fil, &nsamples, 2, &read);
                f_read(&fil, &nsamples, 2, &read);
                break;
            }
            case 0x92:
            {
                f_read(&fil, &data, 1, &read);
                uint32_t streamFreq;
                f_read(&fil, &streamFreq, 4, &read);
                break;
            }
            case 0x93:
            {
                f_read(&fil, &data, 1, &read);
                uint32_t streamFreq;
                f_read(&fil, &streamFreq, 4, &read);
                f_read(&fil, &data, 1, &read);
                f_read(&fil, &streamFreq, 4, &read);
                break;
            }
            case 0x94:
            {
                 f_read(&fil, &data, 1, &read);
                 break;
            }
            case 0x95:
            {
                f_read(&fil, &data, 1, &read);
                f_read(&fil, &nsamples, 2, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }


            case 0xA0:
            case 0xB0:
            case 0xB1:
            case 0xB2:
            case 0xB5: //Ignore common secondary PCM chips
            case 0xB6:
            case 0xB7:
            case 0xB8:
            case 0xB9:
            case 0xBA:
            case 0xBB:
            case 0xBC:
            case 0xBD:
            case 0xBE:
            case 0xBF:
            {
                f_read(&fil, &nsamples, 2, &read);
                break;
            }

            case 0xC0: //24 bit write PCM chips
            case 0xC1:
            case 0xC2:
            case 0xC3:
            case 0xC4:
            case 0xC5:
            case 0xC6:
            case 0xC7:
            case 0xC8:
            case 0xD0:
            case 0xD1:
            case 0xD2:
            case 0xD3:
            case 0xD4:
            case 0xD5:
            case 0xD6:
            {
                f_read(&fil, &data, 1, &read);
                f_read(&fil, &data, 1, &read);
                f_read(&fil, &data, 1, &read);
                break;
            }

            case 0xE0:
            {
                uint32_t dummy;
                f_read(&fil, &dummy, 4, &read);
                break;
            }
            case 0xE1:
            {
                uint32_t dummy;
                f_read(&fil, &dummy, 4, &read);
                break;
            }

            default:
            {
                myprintf("Unhandled case 0x%x\r\n", cmd);
                break;
            }
        }
    }

cleanup:
    f_close(&fil);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  myprintf("Hey dude!" __TIME__ " " __DATE__ "\n");
  myprintf("Running at %ld Hz\n", SystemCoreClock);

  HAL_Delay(1000); //a short delay is important to let the SD card settle


    HAL_GPIO_WriteNamedPin(YM_RD, 1);
    HAL_GPIO_WriteNamedPin(YM_WR, 1);
    HAL_GPIO_WriteNamedPin(YM_CS, 1);
    HAL_GPIO_WriteNamedPin(YM_A0, 0);

  uint32_t ARR_RegisterValue;
  uint32_t PeriodTicks;
  uint32_t Prescalerfactor;
  uint32_t period_cyc;

  period_cyc = SystemCoreClock / 44100;
  Prescalerfactor = (period_cyc / 0x10000) + 1;
  __HAL_TIM_SET_PRESCALER(&htim4, Prescalerfactor - 1);
  PeriodTicks = period_cyc / Prescalerfactor;

  if (PeriodTicks > 0) {
    // The register specifies the maximum value, so the period is really one tick longer
    ARR_RegisterValue = PeriodTicks - 1;
  } else {
    // But do not underflow in case a zero period was given somehow.
    ARR_RegisterValue = 0;
  }
  __HAL_TIM_SET_AUTORELOAD(&htim4, ARR_RegisterValue);

  // Start timer in Time base mode. Required when there is no channel used but only update interrupt.
  HAL_TIM_Base_Start_IT(&htim4);

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
    myprintf("f_mount error (%i)\n", fres);
    while(1);
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
    myprintf("f_getfree error (%i)\n", fres);
    while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\n%10lu KiB total drive space.\n%10lu KiB available.\n", total_sectors / 2, free_sectors / 2);
    FRESULT res;
    DIR dir;
    static FILINFO fno;


  res = f_opendir(&dir, "/");                       /* Open the directory */
  if (res == FR_OK) {
      for (;;) {
          res = f_readdir(&dir, &fno);                   /* Read a directory item */
          if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
          if (fno.fattrib & AM_DIR) {                    /* It is a directory */

          } else {                                       /* It is a file. */
              myprintf("%s\n", fno.fname);
              for(int i = 0; fno.fname[i]; i++){
                fno.fname[i] = tolower(fno.fname[i]);
              }
              if (strstr(fno.fname, ".vgm"))
              {
                process_vgm(fno.fname);
              }
          }
      }
      f_closedir(&dir);
  }



/*

  //Read 30 bytes from "test.txt" on the SD card
  BYTE readBuf[30];

  //Now let's try to open file "test.txt"
  fres = f_open(&fil, "test.txt", FA_READ);
  if (fres != FR_OK) {
    myprintf("f_open error (%i)\n", fres);
    //while(1);
  }
  else
  {
    myprintf("I was able to open 'test.txt' for reading!\n");



    //We can either use f_read OR f_gets to get data out of files
    //f_gets is a wrapper on f_read that does some string formatting for us
    TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
    if(rres != 0) {
    myprintf("Read string from 'test.txt' contents: %s\n", readBuf);
    } else {
    myprintf("f_gets error (%i)\n", fres);
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);
  }

  //Now let's try and write a file "write.txt"
  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
  myprintf("I was able to open 'write.txt' for writing\n");
  } else {
  myprintf("f_open error (%i)\n", fres);
  }

  //Copy in a string
  strncpy((char*)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if(fres == FR_OK) {
  myprintf("Wrote %i bytes to 'write.txt'!\n", bytesWrote);
  } else {
  myprintf("f_write error (%i)\n");
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);


*/


  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);


  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  /* USER CODE END 5 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
