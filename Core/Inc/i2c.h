/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */
#define LOCK_I2C_RESOURCE()          HAL_NVIC_DisableIRQ(ALERT_A_EXTI_IRQn);
#define UNLOCK_I2C_RESOURCE()         HAL_NVIC_EnableIRQ(ALERT_A_EXTI_IRQn);
/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef I2C_Read_USB_PD(uint8_t Port,uint16_t I2cDeviceID_7bit ,uint16_t Address ,void *DataR ,uint16_t Length);
HAL_StatusTypeDef I2C_Write_USB_PD(uint8_t Port,uint16_t I2cDeviceID_7bit ,uint16_t Address ,uint8_t *DataW ,uint16_t Length);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

