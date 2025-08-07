#ifndef __CONTROL_STEP_H__
#define __CONTROL_STEP_H__
#include "main.h"               // Device header
#include <stdbool.h>
#define FORM_LEN 	   20000
#define TIM_PRESCALER   1
#define MOTOR_DIR_CW()	GPIO_SetPins(GPIO_PORT_C,GPIO_PIN_06);		
#define MOTOR_DIR_CCW()	GPIO_ResetPins(GPIO_PORT_C,GPIO_PIN_06);
#define mode 1 //0��ʾ�����ƶ�ģʽ��1��ʾ�Ӽ���ģʽ
typedef struct {
	__IO uint8_t  status;             /* ״̬ */
	__IO uint8_t  dir;                /* ���� */
	__IO uint32_t pos;                /* λ�� */
  __IO uint16_t pluse_time;         /* ����ʱ�� */
}Stepper_Typedef;
/* S�Ӽ������õ��Ĳ��� */
typedef struct {
  __IO int32_t Vo;                  /* ��ʼ�ٶ� */
  __IO int32_t Vt;                  /* ĩ�ٶ� */
  __IO int32_t AccelTotalStep;      /* �����ܲ��� */
  __IO int32_t DecPoint;            /* ��ʼ���ٵĲ��� */
  __IO int32_t TotalStep;           /* ���������ܲ��� */
  __IO int32_t INC_AccelTotalStep;  /* �Ӽ��ٶȲ��� */
  __IO int32_t Dec_AccelTotalStep;  /* �����ٶȲ��� */
  __IO float   Form[FORM_LEN];/* S�Ӽ��� �ٶȱ� */
}SpeedCalc_TypeDef;
/* ����ٶȾ����е��ĸ�״̬ */
typedef enum {
  STOP = 0U,                        /* ֹͣ״̬ */
  ACCEL,                            /* ����״̬ */
  UNIFORM,                          /* ����״̬ */
  DECEL,                            /* ����״̬ */
}StateEnum_TypeDef;
/*Ƶ����ز���*/
#define PWM_TIM 				CM_TMRA_3
#define PWM_TIM_CH 				TMRA_CH3
#define T1_FREQ               (100000000 / (TIM_PRESCALER))//Ƶ��ftֵ
/*�����Ȧ����*/
#define STEP_ANGLE						1.8f									           //��������Ĳ���� ��λ����
#define FSPR              		(360.0f / 1.8f)                  //���������һȦ����������
			
#define MICRO_STEP        		8          				             //ϸ����ϸ���� 
#define SPR               		(FSPR * MICRO_STEP)              //ϸ�ֺ�һȦ����������
#define CONVER(speed)         (float)(speed * SPR / 60.0f)     //���ݵ��ת�٣�r/min�������������٣�step/s��
#define MIN_SPEED             (float)(T1_FREQ / 65535U)        //���Ƶ��/�ٶ�
void App_PortCfg(void);
void Stepper_start(void);
void Stepper_stop(void);
void Speed_Decision_C(int32_t *position);
static bool Calcspeed_e(int32_t Vo,int32_t Vt,float T);
static bool Calcspeed_s(int32_t Vo,int32_t Vt,float T);
void Speed_Decision_S(void);
bool Stepper_Move_S(int16_t start_speed, int16_t end_speed, float acc_time, int32_t step);
void Stepper_Move_C(int16_t speed,int32_t *position,int32_t step);
void set_speed(uint32_t freq);
void set_position(uint16_t	*positon,uint16_t target,uint8_t *command);
extern Stepper_Typedef Stepper;
#endif

