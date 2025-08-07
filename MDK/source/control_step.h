#ifndef __CONTROL_STEP_H__
#define __CONTROL_STEP_H__
#include "main.h"               // Device header
#include <stdbool.h>
#define FORM_LEN 	   20000
#define TIM_PRESCALER   1
#define MOTOR_DIR_CW()	GPIO_SetPins(GPIO_PORT_C,GPIO_PIN_06);		
#define MOTOR_DIR_CCW()	GPIO_ResetPins(GPIO_PORT_C,GPIO_PIN_06);
#define mode 1 //0表示定速移动模式，1表示加减速模式
typedef struct {
	__IO uint8_t  status;             /* 状态 */
	__IO uint8_t  dir;                /* 方向 */
	__IO uint32_t pos;                /* 位置 */
  __IO uint16_t pluse_time;         /* 脉冲时间 */
}Stepper_Typedef;
/* S加减速所用到的参数 */
typedef struct {
  __IO int32_t Vo;                  /* 初始速度 */
  __IO int32_t Vt;                  /* 末速度 */
  __IO int32_t AccelTotalStep;      /* 加速总步数 */
  __IO int32_t DecPoint;            /* 开始减速的步数 */
  __IO int32_t TotalStep;           /* 完整曲线总步数 */
  __IO int32_t INC_AccelTotalStep;  /* 加加速度步数 */
  __IO int32_t Dec_AccelTotalStep;  /* 减加速度步数 */
  __IO float   Form[FORM_LEN];/* S加减速 速度表 */
}SpeedCalc_TypeDef;
/* 电机速度决策中的四个状态 */
typedef enum {
  STOP = 0U,                        /* 停止状态 */
  ACCEL,                            /* 加速状态 */
  UNIFORM,                          /* 匀速状态 */
  DECEL,                            /* 减速状态 */
}StateEnum_TypeDef;
/*频率相关参数*/
#define PWM_TIM 				CM_TMRA_3
#define PWM_TIM_CH 				TMRA_CH3
#define T1_FREQ               (100000000 / (TIM_PRESCALER))//频率ft值
/*电机单圈参数*/
#define STEP_ANGLE						1.8f									           //步进电机的步距角 单位：度
#define FSPR              		(360.0f / 1.8f)                  //步进电机的一圈所需脉冲数
			
#define MICRO_STEP        		8          				             //细分器细分数 
#define SPR               		(FSPR * MICRO_STEP)              //细分后一圈所需脉冲数
#define CONVER(speed)         (float)(speed * SPR / 60.0f)     //根据电机转速（r/min），计算电机步速（step/s）
#define MIN_SPEED             (float)(T1_FREQ / 65535U)        //最低频率/速度
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

