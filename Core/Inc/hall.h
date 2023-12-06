#ifndef __HALL_H__
#define __HALL_H__

#include "stm32f4xx_hal.h"

#define KEY_ON	1
#define KEY_OFF	0


#define HALL_TIMx                        TIM5
#define HALL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM5_CLK_ENABLE()

#define HALL_TIM_GPIO_AF                 GPIO_AF2_TIM5
#define HALL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()

#define HALL_TIM_CH1_PORT                GPIOH          // CH1的引脚
#define HALL_TIM_CH1_PIN                 GPIO_PIN_10
#define HALL_TIM_CH1                     TIM_CHANNEL_1

#define HALL_TIM_CH2_PORT                GPIOH          // CH2的引脚
#define HALL_TIM_CH2_PIN                 GPIO_PIN_11
#define HALL_TIM_CH2                     TIM_CHANNEL_2

#define HALL_TIM_CH3_PORT                GPIOH          // CH3的引脚
#define HALL_TIM_CH3_PIN                 GPIO_PIN_12
#define HALL_TIM_CH3                     TIM_CHANNEL_3

#define HALL_TIM_IRQn                    TIM5_IRQn
#define HALL_TIM_IRQHanler               TIM5_IRQHandler


// 定义定时器预分频，定时器实际时钟频率为：84MHz/（HALL_TIMx_PRESCALER+1）
#define HALL_TIM_PRESCALER               83  // 实际时钟频率为：1MHz

#define HALL_TIM_FREQ                (84e6/(HALL_TIM_PRESCALER+1)) // 定时器计数频率

// 定义定时器周期，当定时器开始计数到HALL_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define HALL_TIM_PERIOD                  0xFFFF //

/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint32_t RT_hallPhase; // 霍尔信号
extern __IO uint32_t RT_hallcomp;  // 霍尔捕获计数值
extern __IO uint32_t RT_hallcnt ; // 霍尔计数值

/* 函数声明 ------------------------------------------------------------------*/
void HALLSensor_TIMx_Init(void);
int32_t HALL_GetPhase(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);


#endif	/* __HALL_H__ */
