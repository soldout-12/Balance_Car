/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
extern volatile uint16_t Tick;
extern volatile uint8_t Key_Flag;
extern uint16_t Button;
extern volatile int16_t speed1;
extern volatile int16_t speed2;
extern volatile int16_t MPU6050_Task_Flag;
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
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_12
#define MPU_INT_GPIO_Port GPIOB
#define MPU_AD0_Pin GPIO_PIN_13
#define MPU_AD0_GPIO_Port GPIOB
#define IIC1_SDA_Pin GPIO_PIN_14
#define IIC1_SDA_GPIO_Port GPIOB
#define IIC1_SCL_Pin GPIO_PIN_15
#define IIC1_SCL_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_8
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_9
#define BIN2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_10
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_11
#define AIN2_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define IIC2_SDA_Pin GPIO_PIN_3
#define IIC2_SDA_GPIO_Port GPIOD
#define IIC2_SCL_Pin GPIO_PIN_4
#define IIC2_SCL_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
