#include "motor.h"
#include "tim.h"
#include "hall.h"

MotorSta_Typedef Motor_State = MOTOR_DISABLE; // ���ʹ��״̬
MotorDir_Typedef Motor_Dir   = MOTOR_DIR_CCW;  // ������� ,˳ʱ��
float PWM_Duty   = 0.25f;        // 25%ռ�ձ�

/**
  * ��������: ��ˢ�����λ����
  * �������: @HALLPhase �����ź���λ
  * �� �� ֵ: ��
  * ˵    ��: ���ƶ�ʱ�����PWM���λ���,���嶨ʱ�����ͨ��CH1ΪA��(U) 
  *           CH2ΪB��(V)��CH3ΪC��(W),������һ�λ��������ʱ��Ĳ���
  */
void BLDCMotor_PhaseCtrl(int32_t HALLPhase )
{
#ifndef USE_PMSMMOTOR  
  /* ˳ʱ��Ļ���˳������ʱ���˳���ǹ���7����,���Կ���ʹ��7����ʱ���˳��õ�
   * ˳ʱ��Ļ���˳��,����ʹ�����������
   */
  if(MOTOR_DIR_CW == Motor_Dir)
    HALLPhase = 0x07 ^ HALLPhase;// ������λ��� 111b ^ 010b -> 101b
#else 
  /* PMSM����ת˳���BLDC�պ����෴��,�趨ΪMOTOR_DIR_CCW��ʱ��BLDC����ʱ����ת
   * PMSM��˳ʱ����ת,�����Ҫ�趨ΪMOTOR_DIR_CCW��ʱ��PMSMΪ��ʱ����ת,��ʹ��
   * ������������������
   */
  if(MOTOR_DIR_CCW == Motor_Dir)
    HALLPhase = 0x07 ^ HALLPhase;// ������λ��� 111b ^ 010b -> 101b
#endif 
  switch(HALLPhase)
  {
    /* ��������U(A),V(B),W(C)����ֱ��Ӧ��CH1,CH2,CH3;
     *  A+,A-�ֱ��ʾCH1���Ƶ���,���ű۵�ͨ
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
  * ��������: �������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ������ˢ���
  */
void  BLDCMotor_Start()
{
  int32_t hallPhase = 0;

  BLDCMOTOR_ENABLE();// ʹ���������PWM
  
  /* ���ű۵�ͨ,���Ծٵ��ݳ�� */
  BLDCMotor_braking_LowerShort();
  HAL_Delay(9); //���ʱ��, ���ֵ ����Ҫ׼ȷ
  
  /* ����HALL�������ӿ��ж� */
  __HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC1);
  HAL_TIMEx_HallSensor_Start_IT(&htim5);
  
  hallPhase = HALL_GetPhase(); // ��ȡ�����ź���λ

  /* ���õ�ǰ�����źŶ�Ӧ��PWM��λ */
  BLDCMotor_PhaseCtrl(hallPhase);  // �������PWM
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM); // �������COM�¼�
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_COM);

  Motor_State = MOTOR_ENABLE;
}

/**
  * ��������: ����ɲ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ����ͣ��,����ֱ���ж����,���������ǵ��ͣ����
  */
void BLDCM_Inertia_brake()
{
  BLDCMOTOR_DISABLE(); // ʹ������оƬ��shutdown�����ж����
  Motor_State = MOTOR_DISABLE;
}
/**
  * ��������: ɲ���ƶ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʹ�����ű�ʹ�����·,�����ܺ��ƶ���һ�ַ�ʽ
  */
void BLDCMotor_braking_LowerShort()
{
  /* �Ƚ�ֹ�ж�,��ֹɲ�����̴���COM�¼�������� */
  HAL_TIMEx_HallSensor_Stop_IT(&htim5);

  /**
    * ֱ�ӹر�MOE,ʹ���ű�����ߵ�ƽ
    * ֱ�ӿ���MOS�����ű۵�ͨ,���ű۹ر�
    */
  /* ���ű۵�ͨ */
  __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH1, 0);// �����Ҫ��ͨ���ű�
  __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH2, 0);// ռ�ձ�����Ϊ100%
  __HAL_TIM_SET_COMPARE(&htim8, BLDCMOTOR_TIM_CH3, 0);// ����
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
  * ��������: ȡ��ɲ���ƶ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ɲ����ʱ��ʹ�����ű۵�ͨ�ķ�ʽ,ȷ��ͣ����֮��ȡ��ɲ���ƶ�ģʽ
  */
void BLDCMotor_unbraking_LS()
{
  /* �ر���� */
  HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH1);
  HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH2);
  HAL_TIM_PWM_Stop(&htim8, BLDCMOTOR_TIM_CH3);
  HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH1);
  HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH2);
  HAL_TIMEx_PWMN_Stop(&htim8, BLDCMOTOR_TIM_CH3);
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);  
}
/**
  * ��������: ���õ��ת��
  * �������: @speed PWM��ռ�ձ�
  * �� �� ֵ: ��
  * ˵    ��: ��
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

