/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define MAIN_OSC_ENABLE_Pin GPIO_PIN_1
#define MAIN_OSC_ENABLE_GPIO_Port GPIOC
#define EN_FB_IN_HIGH_Pin GPIO_PIN_1
#define EN_FB_IN_HIGH_GPIO_Port GPIOA
#define MEAS_3_3_Pin GPIO_PIN_2
#define MEAS_3_3_GPIO_Port GPIOA
#define MEAS_1_6_Pin GPIO_PIN_3
#define MEAS_1_6_GPIO_Port GPIOA
#define VDD_PA_MEAS_Pin GPIO_PIN_4
#define VDD_PA_MEAS_GPIO_Port GPIOA
#define UI_ENABLE_Pin GPIO_PIN_4
#define UI_ENABLE_GPIO_Port GPIOC
#define MAIN_BUCK_EN_Pin GPIO_PIN_5
#define MAIN_BUCK_EN_GPIO_Port GPIOC
#define PHASE_SENSE_OVER_Pin GPIO_PIN_9
#define PHASE_SENSE_OVER_GPIO_Port GPIOD
#define PHASE_SENSE_UNDER_Pin GPIO_PIN_10
#define PHASE_SENSE_UNDER_GPIO_Port GPIOD
#define PHASE_SENSE_ZERO_Pin GPIO_PIN_11
#define PHASE_SENSE_ZERO_GPIO_Port GPIOD
#define EXT_TX_OUT_MODULATOR_Pin GPIO_PIN_13
#define EXT_TX_OUT_MODULATOR_GPIO_Port GPIOD
#define XO_FREQ_SYNC_IN_Pin GPIO_PIN_12
#define XO_FREQ_SYNC_IN_GPIO_Port GPIOA
#define SYS_JTMS_SWDIO_Pin GPIO_PIN_13
#define SYS_JTMS_SWDIO_GPIO_Port GPIOA
#define SYS_JTCK_SWCLK_Pin GPIO_PIN_14
#define SYS_JTCK_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
