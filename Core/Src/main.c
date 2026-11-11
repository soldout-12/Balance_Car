/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "lcd.h" 
#include "key.h"
#include "mpu6050.h"
#include "iic.h" 
#include "tracking.h"
#include "pid_demo.h"
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

/* USER CODE BEGIN PV */
uint16_t volatile Tick;
volatile uint8_t Key_Flag;
volatile uint16_t Second;
volatile int16_t speed1;
volatile int16_t speed2;
volatile int16_t MPU6050_Task_Flag;
volatile uint8_t g_speed_fresh = 0;   /* 编码器10ms刷新标志：速度环消费后清零，保证控制与反馈同相同频 */
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Uart2_DmaRxStart();  /* 启动 DMA+IDLE 接收：数据经 RxEventCallback 进环形队列 */
  /* main() 里，所有 MX_TIMx_Init() 之后调用 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);        /* PA8，电机 A */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);        /* PA9，电机 B */

HAL_TIM_Base_Start_IT(&htim6);                   /* 仅当 Timebase 还是 SysTick 时才加这行 */

HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  /* 编码器 1，PA6/PA7 */
HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);  /* 编码器 2，PC6/PC7 */

pid_Init();   /* 初始化三组PID的 kp/ki/kd/限幅，不调用则增益全为0，PID无输出 */
//循迹初始化
Tracking_Task_Init();


  if (ILI9341_Init() != HAL_OK) {
      uart2_send_string("[E] LCD init fail\r\n");   /* LCD失败不致命，只是没显示，车照跑 */
  }
LCD_FillOrFail(LCD_COLOR_BLACK);
if (MPU6050_Init() != IIC_OK) {
      uart2_send_string("[E] MPU no ACK, check IIC1 wiring & pull-ups\r\n");  /* 不杀车，让校准循环去等 */
    }else  uart2_send_string("[E] MPU_Init ok!\r\n");  /* 不杀车，让校准循环去等 */
   
  /* 校准角速度：MPU不在线时周期性打印提示，不再无声死等（也便于区分"卡住"和"死了"） */
  while (!Calibration_100()) {
      uart2_send_string("[W]等待校准...\r\n");
      LCD_ShowString(0,  0, "wating...:", LCD_COLOR_GREEN, LCD_COLOR_BLACK,32);
      IIC_RecoverBus();   /* 上次运行中途复位可能把MPU晾在半字节、它拉死SDA；打9时钟+STOP解锁 */
  }
  uart2_send_string("[W] 校准完成！\r\n");
 LCD_FillOrFail(LCD_COLOR_WHITE);
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Key_Task();
    /* MPU6050_Task runs in TIM6 ISR (5 ms), not here */
    LCD_Task();
	Uart_Receive_Task();
	Tracking_Task();
  Uart_Transmit_Task();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

//串口发送中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        tx_busy = 0;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    }
}
//串口错误回调：噪声/帧错误/溢出会把 DMA 接收中止掉，必须重启，否则接收悄悄死掉
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);  /* 清PE/FE/ORE标志，防止标志残留导致ISR反复重入 */
        (void)Uart2_DmaRxStart();        /* 重启失败也只是串口暂不可用，下次错误再试，绝不杀车 */
    }
}
//按键3
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press_ms = 0;
    if (GPIO_Pin == GPIO_PIN_13) // 判断是不是 PE13 触发的中断
    {
        /* 200ms 消抖：机械按键一次按下会抖出多个下降沿，
           不消抖 ^= 会把 Key_Flag 抖回原值，现象就是"按了没反应" */
        uint32_t now = HAL_GetTick();
        if (now - last_press_ms < 200) return;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_RESET) // 确认按键真的被按下（低电平）
        {
            last_press_ms = now;
            Key_Flag^=1;
            // const char *msg = (Key_Flag) ? "KEY ON\r\n" : "KEY OFF\r\n";
            // while (tx_busy);           /* 调试用：确认EXTI是否真的触发，调好后可删 */
            // HAL_UART_Transmit(&huart2, (uint8_t *)msg, 8, 20);
           // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
        }
    }
}
//定时器6计时
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
      Tick++;
      
      if(gyro_cal.done && Tick%5==0){MPU6050_Task();}/* run in ISR, skip I2C until calibrated */

       if(Tick%1000==0){
        Second++;
       

        int16_t cnt1=(int16_t)__HAL_TIM_GET_COUNTER(&htim3);//车身参考系左轮E1A/B
        int16_t cnt2=(int16_t)__HAL_TIM_GET_COUNTER(&htim8);//车身参考系右轮E2A/B
        speed1=cnt1;
        speed2=cnt2;
        __HAL_TIM_SET_COUNTER(&htim3,0);
        __HAL_TIM_SET_COUNTER(&htim8,0);
        g_speed_fresh = 1;

       }
    }
}
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
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
