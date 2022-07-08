/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define YM_ICL_Pin GPIO_PIN_14
#define YM_ICL_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define BUS_D0_Pin GPIO_PIN_0
#define BUS_D0_GPIO_Port GPIOA
#define BUS_D1_Pin GPIO_PIN_1
#define BUS_D1_GPIO_Port GPIOA
#define BUS_D2_Pin GPIO_PIN_2
#define BUS_D2_GPIO_Port GPIOA
#define BUS_D3_Pin GPIO_PIN_3
#define BUS_D3_GPIO_Port GPIOA
#define BUS_D4_Pin GPIO_PIN_4
#define BUS_D4_GPIO_Port GPIOA
#define BUS_D5_Pin GPIO_PIN_5
#define BUS_D5_GPIO_Port GPIOA
#define BUS_D6_Pin GPIO_PIN_6
#define BUS_D6_GPIO_Port GPIOA
#define BUS_D7_Pin GPIO_PIN_7
#define BUS_D7_GPIO_Port GPIOA
#define YM_A0_Pin GPIO_PIN_0
#define YM_A0_GPIO_Port GPIOB
#define YM_WR_Pin GPIO_PIN_1
#define YM_WR_GPIO_Port GPIOB
#define YM_CS_Pin GPIO_PIN_2
#define YM_CS_GPIO_Port GPIOB
#define MIDI_TX_Pin GPIO_PIN_10
#define MIDI_TX_GPIO_Port GPIOB
#define MIDI_RX_Pin GPIO_PIN_11
#define MIDI_RX_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOB
#define SD_DET_Pin GPIO_PIN_8
#define SD_DET_GPIO_Port GPIOA
#define USB_DET_Pin GPIO_PIN_15
#define USB_DET_GPIO_Port GPIOA
#define BTN4_Pin GPIO_PIN_4
#define BTN4_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_5
#define BTN3_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_6
#define BTN2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_7
#define BTN1_GPIO_Port GPIOB
#define BTN0_Pin GPIO_PIN_8
#define BTN0_GPIO_Port GPIOB
#define YM_RD_Pin GPIO_PIN_9
#define YM_RD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
