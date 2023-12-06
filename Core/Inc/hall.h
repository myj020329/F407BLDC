#ifndef __HALL_H__
#define __HALL_H__

#include "stm32f4xx_hal.h"

#define KEY_ON	1
#define KEY_OFF	0


#define HALL_TIMx                        TIM5
#define HALL_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM5_CLK_ENABLE()

#define HALL_TIM_GPIO_AF                 GPIO_AF2_TIM5
#define HALL_TIM_GPIO_RCC_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()

#define HALL_TIM_CH1_PORT                GPIOH          // CH1������
#define HALL_TIM_CH1_PIN                 GPIO_PIN_10
#define HALL_TIM_CH1                     TIM_CHANNEL_1

#define HALL_TIM_CH2_PORT                GPIOH          // CH2������
#define HALL_TIM_CH2_PIN                 GPIO_PIN_11
#define HALL_TIM_CH2                     TIM_CHANNEL_2

#define HALL_TIM_CH3_PORT                GPIOH          // CH3������
#define HALL_TIM_CH3_PIN                 GPIO_PIN_12
#define HALL_TIM_CH3                     TIM_CHANNEL_3

#define HALL_TIM_IRQn                    TIM5_IRQn
#define HALL_TIM_IRQHanler               TIM5_IRQHandler


// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��HALL_TIMx_PRESCALER+1��
#define HALL_TIM_PRESCALER               83  // ʵ��ʱ��Ƶ��Ϊ��1MHz

#define HALL_TIM_FREQ                (84e6/(HALL_TIM_PRESCALER+1)) // ��ʱ������Ƶ��

// ���嶨ʱ�����ڣ�����ʱ����ʼ������HALL_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define HALL_TIM_PERIOD                  0xFFFF //

/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint32_t RT_hallPhase; // �����ź�
extern __IO uint32_t RT_hallcomp;  // �����������ֵ
extern __IO uint32_t RT_hallcnt ; // ��������ֵ

/* �������� ------------------------------------------------------------------*/
void HALLSensor_TIMx_Init(void);
int32_t HALL_GetPhase(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);


#endif	/* __HALL_H__ */
