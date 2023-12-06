#include "hall.h"
#include "motor.h"
#include "tim.h"

__IO uint32_t RT_hallPhase = 0; // 霍尔信号相位 RT-> Real Time
uint32_t LS_hallPhase = 0; // 上一次的霍尔信号相位

__IO uint32_t RT_hallcomp = 0;  // 霍尔计数值
__IO uint32_t RT_hallcnt = 0;  // 霍尔计数值

MotorDir_Typedef RT_hallDir = MOTOR_DIR_CW; // 霍尔顺序得到的电机转动方向

/* 霍尔信号顺序,用于确定电机转动方向 
 * 下标是当前的霍尔序列,该下标对应的元素值是上一次的霍尔序列
 * 只要记录上一次的霍尔值,然后对比数据即可知道当前电机实际转动方向
 */
#ifndef USE_PMSMMOTOR 
const uint8_t HallDirCcw [7] = {0, 3, 6, 2, 5, 1, 4};   // 0 无意义
#else 
const uint8_t HallDirCcw [7] = {0, 5, 3, 1, 6, 4, 2};    // PMSM 的逆时针旋转序列
#endif

/**
  * 函数功能: 读取霍尔引脚状态
  * 输入参数: 无
  * 返 回 值: 霍尔引脚状态
  * 说    明: 直接读取引脚的状态,数据字节的低三位分别对应UVW(HALL)的电平状态
  */
int32_t HALL_GetPhase()
{
  int32_t tmp = 0;
  tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN);//U(A)
  tmp <<= 1;
  tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN);//V(B)
  tmp <<= 1;
  tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN);//W(C)
  return (tmp & 0x0007); // 取低三位
}

/**
  * 函数功能: 霍尔传感器回调函数
  * 输入参数: @htim,霍尔传感器接口定时器
  * 返 回 值: 无
  * 说    明:
  */
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//  int32_t RT_hallPhase = 0; // 霍尔信号
//  RT_hallPhase = HALL_GetPhase();     // 获取霍尔引脚的相位
//  /* 换相控制 */
//  BLDCMotor_PhaseCtrl(RT_hallPhase);
//  
//  RT_hallcnt++; // 记录进入中断次数
//  /* 记录时间,假定不会溢出 */
//  RT_hallcomp = __HAL_TIM_GET_COMPARE(htim,HALL_TIM_CH1); 
//  
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  int32_t RT_hallPhase = 0; // 霍尔信号
  RT_hallPhase = HALL_GetPhase();     // 获取霍尔引脚的相位
  /* 换相控制 */
  BLDCMotor_PhaseCtrl(RT_hallPhase);
  
  /* 获取两次信号的计数间隔 *//* 假定不会溢出 */
  RT_hallcomp += __HAL_TIM_GET_COMPARE(htim,HALL_TIM_CH1);
  RT_hallcnt++;
  /* 判断方向 */
  if(HallDirCcw[RT_hallPhase] == LS_hallPhase) // 序列与表中的一致
  {
    RT_hallDir = MOTOR_DIR_CCW; 
  }
  else
    RT_hallDir = MOTOR_DIR_CW;
  LS_hallPhase = RT_hallPhase; // 记录这一个的霍尔值
}



uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*检测是否有按键按下 */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )  
	{	 
		/*等待按键释放 */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}


