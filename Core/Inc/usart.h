/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>  /* 提供 bool、true、false */
#include <stdint.h>   /* 提供 uint8_t、uint16_t、uint32_t */
#include "uart_vofa.h" /* 提供 CH_COUNT 和 fdata 声明 */
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */



extern volatile uint8_t tx_busy ;
typedef struct UartRxAppRing UartRxAppRing;
extern UartRxAppRing uart2_App_ring;       /* USART2 的接收队列 */
extern uint16_t peek_len;
void uart2_send_string(char* str);
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UartRing_PutFromIsr(UartRxAppRing *ring, uint8_t byte); /* 中断侧入队 */
bool UartRing_Get(UartRxAppRing *ring, uint8_t *byte); /* 主循环侧出队 */
uint8_t Uart2_DmaRxStart(void);  /* 返回0=成功；失败不致命，由错误回调重试 */
void Uart_Transmit_Task();
void Uart_Receive_Task();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

