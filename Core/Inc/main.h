/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define USB_GPIO_Pin GPIO_PIN_2
#define USB_GPIO_GPIO_Port GPIOA
#define PDGOOD_Pin GPIO_PIN_3
#define PDGOOD_GPIO_Port GPIOA
#define PD5V_Pin GPIO_PIN_4
#define PD5V_GPIO_Port GPIOA
#define PD9V_Pin GPIO_PIN_5
#define PD9V_GPIO_Port GPIOA
#define PD15V_Pin GPIO_PIN_6
#define PD15V_GPIO_Port GPIOA
#define PD20V_Pin GPIO_PIN_7
#define PD20V_GPIO_Port GPIOA
#define DISABLE_N12_Pin GPIO_PIN_0
#define DISABLE_N12_GPIO_Port GPIOB
#define DISABLE_5P0_Pin GPIO_PIN_1
#define DISABLE_5P0_GPIO_Port GPIOB
#define ATTACH_Pin GPIO_PIN_2
#define ATTACH_GPIO_Port GPIOB
#define USBPD_ALERT_Pin GPIO_PIN_12
#define USBPD_ALERT_GPIO_Port GPIOB
#define USBPD_ALERT_EXTI_IRQn EXTI15_10_IRQn
#define ALERT_5V_Pin GPIO_PIN_13
#define ALERT_5V_GPIO_Port GPIOB
#define ALERT_5V_EXTI_IRQn EXTI15_10_IRQn
#define ALERT_NEG_12V_Pin GPIO_PIN_14
#define ALERT_NEG_12V_GPIO_Port GPIOB
#define ALERT_NEG_12V_EXTI_IRQn EXTI15_10_IRQn
#define ALERT_POS_12V_Pin GPIO_PIN_15
#define ALERT_POS_12V_GPIO_Port GPIOB
#define ALERT_POS_12V_EXTI_IRQn EXTI15_10_IRQn
#define DISABLE_PRI_12V_Pin GPIO_PIN_8
#define DISABLE_PRI_12V_GPIO_Port GPIOA
#define PGOOD_12V_Pin GPIO_PIN_9
#define PGOOD_12V_GPIO_Port GPIOA
#define PGOOD_5V_Pin GPIO_PIN_10
#define PGOOD_5V_GPIO_Port GPIOA
#define PDBAD_Pin GPIO_PIN_15
#define PDBAD_GPIO_Port GPIOA
#define USBPD_RST_Pin GPIO_PIN_3
#define USBPD_RST_GPIO_Port GPIOB
#define PWR_BTN_Pin GPIO_PIN_5
#define PWR_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
