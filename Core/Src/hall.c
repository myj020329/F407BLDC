#include "hall.h"
#include "motor.h"
#include "tim.h"

__IO uint32_t RT_hallPhase = 0; // �����ź���λ RT-> Real Time
uint32_t LS_hallPhase = 0; // ��һ�εĻ����ź���λ

__IO uint32_t RT_hallcomp = 0;  // ��������ֵ
__IO uint32_t RT_hallcnt = 0;  // ��������ֵ

MotorDir_Typedef RT_hallDir = MOTOR_DIR_CW; // ����˳��õ��ĵ��ת������

/* �����ź�˳��,����ȷ�����ת������ 
 * �±��ǵ�ǰ�Ļ�������,���±��Ӧ��Ԫ��ֵ����һ�εĻ�������
 * ֻҪ��¼��һ�εĻ���ֵ,Ȼ��Ա����ݼ���֪����ǰ���ʵ��ת������
 */
#ifndef USE_PMSMMOTOR 
const uint8_t HallDirCcw [7] = {0, 3, 6, 2, 5, 1, 4};   // 0 ������
#else 
const uint8_t HallDirCcw [7] = {0, 5, 3, 1, 6, 4, 2};    // PMSM ����ʱ����ת����
#endif

/**
  * ��������: ��ȡ��������״̬
  * �������: ��
  * �� �� ֵ: ��������״̬
  * ˵    ��: ֱ�Ӷ�ȡ���ŵ�״̬,�����ֽڵĵ���λ�ֱ��ӦUVW(HALL)�ĵ�ƽ״̬
  */
int32_t HALL_GetPhase()
{
  int32_t tmp = 0;
  tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN);//U(A)
  tmp <<= 1;
  tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN);//V(B)
  tmp <<= 1;
  tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN);//W(C)
  return (tmp & 0x0007); // ȡ����λ
}

/**
  * ��������: �����������ص�����
  * �������: @htim,�����������ӿڶ�ʱ��
  * �� �� ֵ: ��
  * ˵    ��:
  */
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//  int32_t RT_hallPhase = 0; // �����ź�
//  RT_hallPhase = HALL_GetPhase();     // ��ȡ�������ŵ���λ
//  /* ������� */
//  BLDCMotor_PhaseCtrl(RT_hallPhase);
//  
//  RT_hallcnt++; // ��¼�����жϴ���
//  /* ��¼ʱ��,�ٶ�������� */
//  RT_hallcomp = __HAL_TIM_GET_COMPARE(htim,HALL_TIM_CH1); 
//  
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  int32_t RT_hallPhase = 0; // �����ź�
  RT_hallPhase = HALL_GetPhase();     // ��ȡ�������ŵ���λ
  /* ������� */
  BLDCMotor_PhaseCtrl(RT_hallPhase);
  
  /* ��ȡ�����źŵļ������ *//* �ٶ�������� */
  RT_hallcomp += __HAL_TIM_GET_COMPARE(htim,HALL_TIM_CH1);
  RT_hallcnt++;
  /* �жϷ��� */
  if(HallDirCcw[RT_hallPhase] == LS_hallPhase) // ��������е�һ��
  {
    RT_hallDir = MOTOR_DIR_CCW; 
  }
  else
    RT_hallDir = MOTOR_DIR_CW;
  LS_hallPhase = RT_hallPhase; // ��¼��һ���Ļ���ֵ
}



uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*����Ƿ��а������� */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )  
	{	 
		/*�ȴ������ͷ� */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}


