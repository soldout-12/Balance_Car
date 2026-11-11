/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <string.h>
#include <stdio.h>
/* USER CODE BEGIN 0 */
#include "uart_vofa.h"
#define UART2_DMA_RX_SIZE 256U /* DMA 接收数组大小；循环模式下会反复回绕 */
#define UART2_APP_RING_SIZE 128U /* 容量取 2 的幂，便于用位运算回绕 */
#define VOFA_PERIOD_MS     20U   /* VOFA 帧发送间隔，20ms≈50Hz */
static uint8_t uart2_dma_rx[UART2_DMA_RX_SIZE]; /* DMA 直接写入的内存数组 */
static uint16_t uart2_dma_old_pos;              /* 上一次回调处理到的写位置 */
void UART2_Print_IT(uint8_t val);

struct UartRxAppRing{                    /* 定义接收环形队列的所有状态 */
    uint8_t data[UART2_APP_RING_SIZE]; /* 实际存放收到的字节 */
    volatile uint16_t head;          /* 生产者写指针：下一个空槽位 */
    volatile uint16_t tail;          /* 消费者读指针：下一个待取字节 */
    volatile uint32_t dropped;       /* 满载时丢弃的字节数，便于诊断 */
};

 UartRxAppRing uart2_App_ring;       /* USART2 的接收队列 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

volatile uint8_t tx_busy = 0;




/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* CubeMX 重新生成会把 MspInit 里的 DMA 模式刷回 NORMAL（一次性接收，随后静默停摆，
     表现为串口只能解析一帧）。在此强制改回 CIRCULAR：环形接收 DMA 常驻，IDLE 逐帧回调。
     放在 USER CODE 段，重新生成不会丢；此时 DMA 尚未使能，重初始化安全。 */
  huart2.hdmarx->Init.Mode = DMA_CIRCULAR;
  HAL_DMA_Init(huart2.hdmarx);
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//出队
bool UartRing_Get(UartRxAppRing *ring, uint8_t *byte) /* 主循环侧出队 */
{
    if (ring->tail == ring->head) {
        return false; /* 读写指针相同表示当前为空 */
    }

    *byte = ring->data[ring->tail]; /* 取出最早到达的字节 */
    ring->tail = (uint16_t)((ring->tail + 1U) & (UART2_APP_RING_SIZE - 1U));
    return true; /* tail 前移后，该槽位重新可写 */
}
//DMA调用入队//DMA缓冲区入队到环形缓冲区
static void RingBuffer_Write(UartRxAppRing *ring,const uint8_t *data, uint16_t length) /* DMA 数据入口 */
{
    for (uint16_t i = 0U; i < length; ++i) { /* 遍历本次新增的连续数据 */
        /* 预计算写入后的位置；按位与等价于对 128 取模。 */
    uint16_t next = (uint16_t)((ring->head + 1U) & (UART2_APP_RING_SIZE - 1U));

    if (next == ring->tail) {
        ++ring->dropped;       /* next 追上 tail，队列已满，保留旧数据 */
        return;
    }

    ring->data[ring->head] = data[i]; /* 先写数据，避免消费者读到半成品 */
    ring->head = next;             /* 最后发布新 head，通知消费者 */         /* 只入队，不在回调中解析或发送 */

    }
}
// 获取环形缓冲区当前可读的数据长度
uint16_t RingBuffer_GetCount(UartRxAppRing *ring) {
    return (ring->head - ring->tail + UART2_APP_RING_SIZE) % UART2_APP_RING_SIZE;
}
// 从环形缓冲区读取指定长度到线性数组（不改变读指针，仅预览）
void RingBuffer_Peek(UartRxAppRing *ring, uint8_t *dest, uint16_t len) {
    uint16_t current_tail = ring->tail;
    for (uint16_t i = 0; i < len; i++) {
        dest[i] = ring->data[current_tail];
        current_tail = (current_tail + 1) % UART2_APP_RING_SIZE;
    }
}
// 更新读指针（跳过已解析的数据）
void RingBuffer_ReadSkip(UartRxAppRing *ring, uint16_t len) {
    ring->tail = (ring->tail + len) % UART2_APP_RING_SIZE;
}


//启动DMA，在main中调用一次；返回0=成功。失败不致命（常见为BUSY），由错误回调再重试
uint8_t Uart2_DmaRxStart(void) /* 启动 USART2 的 DMA+IDLE 接收 */
{
    uart2_dma_old_pos = 0U; /* 新一轮接收从数组下标 0 开始计算增量 */
    /* 注意：这里绝不能进 Error_Handler——运行中一次串口误帧就会走到这，
       把调试链路的错误升级成整车死亡是错的。挂载失败返回错误码即可。 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, /* 指定 USART2 句柄 */
                                    uart2_dma_rx, /* 指定 DMA 写入的数组首地址 */
                                    UART2_DMA_RX_SIZE) != HAL_OK) { /* 指定数组容量并启用 IDLE 回调 */
        return 1U;
    }

    /* 关闭半传输中断，避免收到半缓冲时提前触发一次回调。 */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); /* 保留传输完成和 IDLE 事件 */
    return 0U;
}
//DMA事件回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) /* IDLE/ DMA 完成事件入口 */
{
    if (huart->Instance != USART2) { /* 该回调可能被其他 USART 共用 */
        return;                       /* 不是 USART2 就不处理 */
    }

    if (size > uart2_dma_old_pos) { /* 写指针未回绕：新数据是一段连续区间 */
        RingBuffer_Write(&uart2_App_ring,&uart2_dma_rx[uart2_dma_old_pos], /* 从旧位置开始读取 */
                            (uint16_t)(size - uart2_dma_old_pos)); /* 处理未回绕的新增长度 */
    } else if (size < uart2_dma_old_pos) { /* 写指针回到数组开头，数据被分成两段 */
        RingBuffer_Write(&uart2_App_ring,&uart2_dma_rx[uart2_dma_old_pos], /* 从旧位置读到数组末尾 */
                            (uint16_t)(UART2_DMA_RX_SIZE - uart2_dma_old_pos)); /* 先处理回绕前的尾部数据 */
        if (size > 0U) {
            RingBuffer_Write(&uart2_App_ring,&uart2_dma_rx[0], size); /* 再处理开头到新位置的数据 */
        }
    }

    uart2_dma_old_pos = size; /* 记录本次已经处理到的 DMA 写位置 */
    if (uart2_dma_old_pos == UART2_DMA_RX_SIZE) { /* 传输完成时 size 可能等于数组长度 */
        uart2_dma_old_pos = 0U;                  /* 下次从数组开头继续计算 */
    }
    
}

uint16_t peek_len;
//主循环回显：从环形队列取多少发回多少
void Uart_Receive_Task(void){
uint16_t available = RingBuffer_GetCount(&uart2_App_ring);
    if (available == 0) return;

    // 注意：parse_buf 需要大一点，因为字符串指令可能较长
    uint8_t parse_buf[256]; 
     peek_len = (available > sizeof(parse_buf) - 1) ? sizeof(parse_buf) - 1 : available;
    
    // 安全拷贝
    RingBuffer_Peek(&uart2_App_ring, parse_buf, peek_len);
    parse_buf[peek_len] = '\0'; // ⚠️ 极其重要：为字符串解析添加结束符，防止 sscanf 越界读取内存

    // 2. 尝试解析字符串调参指令 (PID 调参)
    // 注意：在实际工程中，最好只开启一种解析器，或者根据数据的特征（例如首字节是否为 0x00 或字母）来分流
    // 为了安全，如果上方 JustFloat 消耗了数据，这里就不再用刚消耗过的数据解析指令
    // 这里演示独立调用：
    uint16_t consumed_cmd = Command_Parser(parse_buf , peek_len );
    if (consumed_cmd > 0) {
        RingBuffer_ReadSkip(&uart2_App_ring, consumed_cmd);
    }
}
/* FireWater 以换行为帧尾，通道状态必须全部放在同一帧里。
   不再周期性插入 ASCII 文本，避免上位机把它解析成单通道采样帧。 */
void Uart_Transmit_Task(void){
    static uint32_t last_vofa_ms = 0;
    uint32_t now = HAL_GetTick();

    if (tx_busy) return;

    if ((uint32_t)(now - last_vofa_ms) >= VOFA_PERIOD_MS) {
        last_vofa_ms = now;
        VOFA_T_Fire_Water();
    }
}
void UART2_Print_IT(uint8_t val)
{
	/* IT 发送是异步的，缓冲区必须在发完前一直有效；
	   放栈上会被主循环后续调用覆盖，发出去的是乱码 */
	static char ascii_buf[8];   // 存转好的字符串，足够0~255
    if(tx_busy) return; // 上一次还没发完，直接丢弃，防止堵控制循环

    // 把数字转ASCII字符串
    sprintf(ascii_buf,"%d\r\n",val);
    uint16_t len = strlen(ascii_buf);

    if(HAL_UART_Transmit_IT(&huart2, (uint8_t*)ascii_buf, len) == HAL_OK)
    {
        tx_busy = 1;
    }
}
void uart2_send_char(char ch)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xffff);
}
void uart2_send_string(char* str)
{
	while (tx_busy);   /* VOFA的IT发送占用串口时，阻塞发会直接被HAL_BUSY丢弃，先等它发完 */
	while(*str!=0&&str!=0)
	{
		uart2_send_char(*str++);
	}
}
/* USER CODE END 1 */













