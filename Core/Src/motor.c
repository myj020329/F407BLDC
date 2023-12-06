#include "motor.h"
#include "tim.h"
#include "hall.h"

MotorSta_Typedef Motor_State = MOTOR_DISABLE; // 电机使能状态
MotorDir_Typedef Motor_Dir   = MOTOR_DIR_CCW;  // 电机方向 ,顺时针
float PWM_Duty   = 0.25f;        // 25%占空比

/**
  * 函数功能: 无刷电机相位控制
  * 输入参数: @HALLPhase 霍尔信号相位
  * 返 回 值: 无
  * 说    明: 控制定时器输出PWM波形换相,定义定时器输出通道CH1为A相(U) 
  *           CH2为B相(V)，CH3为C相(W),配置下一次霍尔换相的时候的波形
  */
void BLDCMotor_PhaseCtrl(int32_t HALLPhase )
{
#ifndef USE_PMSMMOTOR  
  /* 顺时针的霍尔顺序与逆时针的顺序是关于7互补,所以可以使用7减逆时针的顺序得到
   * 顺时针的霍尔顺序,这里使用异或代替减法
   */
  if(MOTOR_DIR_CW == Motor_Dir)
    HALLPhase = 0x07 ^ HALLPhase;// 将低三位异或 111b ^ 010b -> 101b
#else 
  /* PMSM的旋转顺序跟BLDC刚好是相反的,设定为MOTOR_DIR_CCW的时候BLDC是逆时针旋转
   * PMSM是顺时针旋转,如果需要设定为MOTOR_DIR_CCW的时候PMSM为逆时针旋转,则使用
   * 下面语句代替上面的语句
   */
  if(MOTOR_DIR_CCW == Motor_Dir)
    HALLPhase = 0x07 ^ HALLPhase;// 将低三位异或 111b ^ 010b -> 101b
#endif 
  switch(HALLPhase)
  {
    /* 定义电机的U(A),V(B),W(C)三相分别对应是CH1,CH2,CH3;
     *  A+,A-分别表示CH1控制的上,下桥臂导通
     */
    case 5: //B+  A-
    {
      /*  Channe3 configuration */ 
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH3);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH3);
    
      /*  Channe2 configuration  */
      __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH2,BLDCMOTOR_TIM_PERIOD * PWM_Duty);
      HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH2);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH2);  
      
      /*  Channe1 configuration */
      __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH1,BLDCMOTOR_TIM_PERIOD +1);
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH1);
      HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH1);
    }
    break;
    
    case 4:// C+ A-
    {
      /*  Channe2 configuration */ 
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH2);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH2);
 
      /*  Channe3 configuration  */
      __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH3,BLDCMOTOR_TIM_PERIOD * PWM_Duty);
      HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH3);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH3);  
      
      /*  Channe1 configuration  */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH1,BLDCMOTOR_TIM_PERIOD +1);
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH1);
      HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH1);
    }
      break;
    
    case 6://C+ B-
    {
      /*  Channe1 configuration  */ 
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH1);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH1);
    
      /*  Channe3 configuration */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH3,BLDCMOTOR_TIM_PERIOD * PWM_Duty);
      HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH3);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH3);  

      /*  Channe2 configuration  */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH2,BLDCMOTOR_TIM_PERIOD +1);
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH2);
      HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH2);
    }
      break;

    case 2: // A+ B-
    {
      /*  Channe3 configuration */       
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH3);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH3);
    
      /*  Channe1 configuration */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH1,BLDCMOTOR_TIM_PERIOD * PWM_Duty);
      HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH1);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH1);  
      
      /*  Channe2 configuration */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH2,BLDCMOTOR_TIM_PERIOD +1);
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH2);
      HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH2);
    }
      break;
    
    
    case 3:// A+ C-
    {
      /*  Channe2 configuration */ 
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH2);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH2);

      /*  Channe1 configuration */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH1,BLDCMOTOR_TIM_PERIOD * PWM_Duty);
      HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH1);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH1);  
      
      /*  Channe3 configuration */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH3,BLDCMOTOR_TIM_PERIOD +1);    
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH3);
      HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH3);
    }
      break;
    case 1: // B+ C-
    {
      /*  Channe1 configuration */ 
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH1);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH1);
    
      /*  Channe2 configuration */
      __HAL_TIM_SET_COMPARE(&htim8,BLDCMOTOR_TIM_CH2, BLDCMOTOR_TIM_PERIOD * PWM_Duty);
      HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH2);
      HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH2);  
      
      /*  Channe3 configuration */
      __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH3, BLDCMOTOR_TIM_PERIOD +1);
      HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH3);
      HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH3);
    }
    break;
  }
}
/**
  * 函数功能: 启动电机
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 启动无刷电机
  */
void  BLDCMotor_Start()
{
  int32_t hallPhase = 0;

  BLDCMOTOR_ENABLE();// 使能允许输出PWM
  
  /* 下桥臂导通,给自举电容充电 */
  BLDCMotor_braking_LowerShort();
  HAL_Delay(9); //充电时间, 大概值 不需要准确
  
  /* 启动HALL传感器接口中断 */
  __HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC1);
  HAL_TIMEx_HallSensor_Start_IT(&htim5);
  
  hallPhase = HALL_GetPhase(); // 获取霍尔信号相位

  /* 配置当前霍尔信号对应的PWM相位 */
  BLDCMotor_PhaseCtrl(hallPhase);  // 配置输出PWM
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM); // 软件生成COM事件
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_COM);

  Motor_State = MOTOR_ENABLE;
}

/**
  * 函数功能: 惯性刹车
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 自由停机,就是直接切断输出,依靠惯性是电机停下来
  */
void BLDCM_Inertia_brake()
{
  BLDCMOTOR_DISABLE(); // 使用驱动芯片的shutdown引脚切断输出
  Motor_State = MOTOR_DISABLE;
}
/**
  * 函数功能: 刹车制动
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 使用下桥臂使电机短路,属于能耗制动的一种方式
  */
void BLDCMotor_braking_LowerShort()
{
  /* 先禁止中断,防止刹车过程触发COM事件重新输出 */
  HAL_TIMEx_HallSensor_Stop_IT(&htim5);

  /**
    * 直接关闭MOE,使下桥臂输出高电平
    * 直接控制MOS管下桥臂导通,上桥臂关闭
    */
  /* 下桥臂导通 */
  __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH1, 0);// 如果需要导通上桥臂
  __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH2, 0);// 占空比设置为100%
  __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH3, 0);// 即可
  HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH1);
  HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH2);
  HAL_TIM_PWM_Start(&htim8, BLDCMOTOR_TIM_CH3);
  HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH1);
  HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH2);
  HAL_TIMEx_PWMN_Start(&htim8, BLDCMOTOR_TIM_CH3);
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);  
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_COM);

  Motor_State = MOTOR_DISABLE;
}
/**
  * 函数功能: 取消刹车制动
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 刹车的时候使用下桥臂导通的方式,确认停下来之后取消刹车制动模式
  */
void BLDCMotor_unbraking_LS()
{
  /* 关闭输出 */
  HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH1);
  HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH2);
  HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH3);
  HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH1);
  HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH2);
  HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH3);
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);  
}
/**
  * 函数功能: 设置电机转速
  * 输入参数: @speed PWM的占空比
  * 返 回 值: 无
  * 说    明: 无
  */
void BLDCMotor_SetSpeed(float speed)
{

  if(speed > 1.0f)
  {
    speed = 1.0f;
  }
  else if(speed < 0.0f)
  {
    speed = 0.0f;
  }
  PWM_Duty = speed;
}

