/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hall.h"
#include "motor.h"
#include "stdio.h"
#include <math.h>

#define TIMECNT   800

uint8_t i = 0;

float MotorSpeed  = 0.0f ;// 电机转速,这里是占空比,
_Bool isTimeUp    = 0;     // 计时标记
uint32_t timeTick = 0;   // 计时标记
float Speed_hz    = 0 ;

extern MotorSta_Typedef Motor_State; // 电机使能状态
extern MotorDir_Typedef Motor_Dir;  // 电机方向 ,顺时针
extern float PWM_Duty;        // 25%占空比

extern MotorDir_Typedef RT_hallDir; // 霍尔顺序得到的电机转动方向


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
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    Motor_Dir = MOTOR_DIR_CW;
//	MotorSpeed = 0.50f;
//    BLDCMotor_SetSpeed(MotorSpeed);
//    BLDCMotor_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   //启动电机设置占空比15%
	if(Key_Scan(KEY1_GPIO_Port,KEY1_Pin) == KEY_ON)
	{
	  MotorSpeed = 0.15f;
      BLDCMotor_SetSpeed(MotorSpeed);
      BLDCMotor_Start();
      printf("按键KEY1启动电机\n");
	}
	
	//增大占空比
	if( Key_Scan(KEY2_GPIO_Port,KEY2_Pin) == KEY_ON)
	{
		MotorSpeed += 0.05f;
      BLDCMotor_SetSpeed(MotorSpeed);
      printf("电机加速\n");
	}
	
	//减小占空比
	if( Key_Scan(KEY3_GPIO_Port,KEY3_Pin) == KEY_ON)
	{
		 MotorSpeed -= 0.05f;
      BLDCMotor_SetSpeed(MotorSpeed);
      printf("电机减速\n");
	}
	
	//控制电机方向
	if( Key_Scan(KEY4_GPIO_Port,KEY4_Pin) == KEY_ON)
	{
		/* 必须先停止,然后再启动 */
      if(Motor_State == MOTOR_DISABLE)
      {
        if(Motor_Dir == MOTOR_DIR_CW)
          Motor_Dir = MOTOR_DIR_CCW;
        else 
          Motor_Dir = MOTOR_DIR_CW;
        printf("需要重新启动电机\n");
      }
      else
        printf("电机更换方向,必须先停止\n");
	}
	
	
	//停止电机
	if( Key_Scan(KEY5_GPIO_Port,KEY5_Pin) == KEY_ON)
	{
	  BLDCMotor_braking_LowerShort();
      HAL_Delay(100);// 等100ms, 电机停下来之后才取消刹车控制
      BLDCMotor_unbraking_LS();
      printf("刹车停止\n");
	}
	
//	if( isTimeUp )
//    {
//      /* 捕获得到的霍尔信号频率 */
//      if( RT_hallcomp == 0 )
//        Speed_hz = 0;
//      else
//        Speed_hz = (float)(HALL_TIM_FREQ)/(float)(RT_hallcomp);
//      RT_hallcomp = 0;
//      
//      /* 霍尔信号的频率,转速rps,转速rpm */
//      printf("%.3f Hz, %.2f RPS, %.2fRPM\n",Speed_hz, Speed_hz/24, (Speed_hz/24)*60);

//      isTimeUp = 0;
//      timeTick = TIMECNT;
//    }
	
	if( isTimeUp )
    {
      uint32_t tmpCC = 0;
      if(RT_hallcnt == 0) // 避免除数为0
      {
        Speed_hz = 0;
      }
      else 
      {
        tmpCC = RT_hallcomp / RT_hallcnt; // tmpCC:两次捕获之间的捕获值,
        Speed_hz = (float)HALL_TIM_FREQ/(float)(tmpCC);
      }
      RT_hallcomp = 0;
      RT_hallcnt  = 0;
      /* 霍尔信号的频率,转速rps,转速rpm */
      if(RT_hallDir == MOTOR_DIR_CW)
        Speed_hz = fabs(Speed_hz);
      else
        Speed_hz = -fabs(Speed_hz);
      /* 未做任何滤波的速度值 */
      printf("%.3f Hz, %.2f RPS, %.2fRPM\n", Speed_hz, Speed_hz/24, (Speed_hz/24)*60);
      
      isTimeUp = 0;
      timeTick = TIMECNT;
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  // HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
  // HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
  // HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);               // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback()
{
  if(timeTick != 0)
    timeTick--;
  if(timeTick == 0)
  {
    isTimeUp = 1;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
