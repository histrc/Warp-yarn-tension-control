#include "control_step.h"
#include <math.h>
#include <string.h>
#define FALSE 0
#define TRUE 1
/* �㷨��ؽṹ��������� */
uint8_t ecode=0;//1,���泤�Ȳ��㡣2���Ӽ��ٲ�������
int32_t n=0;//���ټ���
int32_t ns=0;//s�μ���
int8_t dir_=0; //��ָʾת�����򣬲��������壩1��ʾ˳ʱ�룬-1��ʾ��ʱ��
SpeedCalc_TypeDef Speed;
Stepper_Typedef Stepper = {0};
uint8_t cmd;//����״̬��־λ
//Port Config
#define STEP_EN_PORT 		GPIO_PORT_C //�������EN���Ŷ���
#define STEP_EN_PIN 		GPIO_PIN_07
#define STEP_DIR_PORT 		GPIO_PORT_C//�������DIR���Ŷ���
#define STEP_DIR_PIN 		GPIO_PIN_06
#define LED_PORT 			GPIO_PORT_B // ����LED��
#define LED_PIN 			GPIO_PIN_12
void App_PortCfg(void)
{
    /* GPIO initialize */
    stc_gpio_init_t stcGpioInit;
    /* PC7 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(STEP_EN_PORT,STEP_EN_PIN, &stcGpioInit);//EN���Ŷ���

    /* PC6 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(STEP_DIR_PORT,STEP_DIR_PIN, &stcGpioInit);//DIR���Ŷ���

    /* PB12 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(LED_PORT, LED_PIN, &stcGpioInit);//LED���Ŷ���

    GPIO_SetFunc(GPIO_PORT_C,GPIO_PIN_00,GPIO_FUNC_42);//SPI1-SS0
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_00,GPIO_FUNC_43);//SPI1-SCK
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_02,GPIO_FUNC_40);//SPI1-MOSI
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_03,GPIO_FUNC_41);//SPI1-MISO
    
    GPIO_SetFunc(GPIO_PORT_C,GPIO_PIN_08,GPIO_FUNC_4);//TIMA-3-PWM3
    
}
/**
  * @brief  ʹ�ܲ������
	*	@note 	��
  */
void Stepper_start(void){
	GPIO_SetPins(STEP_EN_PORT,STEP_EN_PIN);//ENABLE4
}
/**
  * @brief  �رղ������ʹ��
	*	@note 	��
  */
void Stepper_stop(void){
	GPIO_ResetPins(STEP_EN_PORT,STEP_EN_PIN);//ENABLE4
}
/**
  * @brief  ָ���������ٶȱ���� 
  * @param  Vo�����ٶ�
  * @param  Vt��ĩ�ٶ�
  * @param  T_s������ʱ��
  * @retval 0��������1����������
	*	@note 	��
  */
static bool Calcspeed_e(int32_t Vo_rpm, int32_t Vt_rpm, float T_s)
{
    /* ------------------ 1. ����ת�� ------------------ */
    float Vo = CONVER(Vo_rpm);          // ��/��
    float Vt = CONVER(Vt_rpm);
    float T  = T_s * 1000.0f;           // ת�� ms�����水 1ms ������
    float tau = T / 3.0f;               // ָ��ʱ�䳣�����ɵ�

    /* ------------------ 2. �����ܲ��� ------------------ */
    float total_dist = (Vo + Vt) * 0.5f * T * 0.001f;   // �����������
    Speed.AccelTotalStep = (int32_t)(total_dist + 0.5f);
    if (Speed.AccelTotalStep % 2) Speed.AccelTotalStep++;
    if (FORM_LEN < Speed.AccelTotalStep) { ecode = 1; return false; }
    /* ------------------ 3. ����ָ���ٶȱ� --------------- */
    for (int32_t i = 0; i < Speed.AccelTotalStep; i++) {
        float t = (float)i * T / Speed.AccelTotalStep;   // ��ǰ���ʱ��
        float v = Vo + (Vt - Vo) * (1.0f - expf(-t / tau));
        Speed.Form[i] = v;
        if (Speed.Form[i] < MIN_SPEED) Speed.Form[i] = MIN_SPEED;
    }

    /* ���ٱ�����ٱ���ȫ�Գƣ�ֱ�ӷ���������� */
    Speed.INC_AccelTotalStep = Speed.AccelTotalStep / 2;
    Speed.Dec_AccelTotalStep = Speed.AccelTotalStep - Speed.INC_AccelTotalStep;
    return true;
}
static bool Calcspeed_s(int32_t Vo,int32_t Vt,float T){
	int32_t i = 0;
	float Vm =0.0f;                // �м���ٶ�
	float K = 0.0f;                // �Ӽ��ٶ�
	float Ti = 0.0f;               // ʱ���� dt
	float Sumt = 0.0f;             // ʱ���ۼ���
	float DeltaV = 0.0f;           // �ٶȵ�����dv
	float temp = 0.0f;             // �м����
	/***************************************************************************/
	/* �������ĩ�ٶ� */
	Speed.Vo = CONVER(Vo);
	Speed.Vt = CONVER(Vt);
	/***************************************************************************/
	/* �����ʼ���� */
	T = T / 2.0f;						//�Ӽ��ٶε�ʱ�䣨���ٶ�б��>0��ʱ�䣩
	Vm = (Speed.Vo + Speed.Vt) / 2.0f;	//�����е���ٶ�
	K = fabsf((2.0f * (Vm - Speed.Vo)) / (T * T));// �����е��ٶȼ���Ӽ��ٶ�	
	Speed.INC_AccelTotalStep = (int32_t)((Speed.Vo * T) + ((K * T * T* T) / 6.0f));// �Ӽ�����Ҫ�Ĳ���(int32_t)((Speed.Vo * T) + ((K * T * T* T) / 6.0f));
	Speed.Dec_AccelTotalStep = (int32_t)(((Speed.Vt + Speed.Vo) * T - Speed.INC_AccelTotalStep));// ��������Ҫ�Ĳ��� S = Vt * Time - S1  (int32_t)(((Speed.Vt + Speed.Vo) * T - Speed.INC_AccelTotalStep))
	/***************************************************************************/
	/* ���㹲��Ҫ�Ĳ�������У���ڴ��С�������ڴ�ռ����ٶȱ� */
	Speed.AccelTotalStep = Speed.Dec_AccelTotalStep + Speed.INC_AccelTotalStep;              // ������Ҫ�Ĳ��� 
	if(Speed.AccelTotalStep  % 2 != 0)     // ���ڸ���������ת�����������ݴ��������,���������1
		Speed.AccelTotalStep  += 1;
	/* �ж��ڴ泤�� */
	if(FORM_LEN < Speed.AccelTotalStep)
	{
		ecode=1;
		//printf("FORM_LEN ���泤�Ȳ���\r\n���뽫 FORM_LEN �޸�Ϊ %d \r\n", Speed.AccelTotalStep);
		return false;
	}
	/***************************************************************************/
	/* �����һ����ʱ�� */
	/* ���ݵ�һ����ʱ����㣬��һ�����ٶȺ�����ʱ���� */
	/* ����λ��Ϊ0��ʱ������������ʱ��Ĺ�ϵʽ -> ����λ�ƺ�ʱ��Ĺ�ʽS = 1/2 * K * Ti^3  �ɵ� Ti=6 * 1 / K��1/3�η� */
	Ti = pow(6.0f * 1.0f / K, 1.0f / 3.0f); //������� Ti ʱ�䳣��
	Sumt += Ti;//�ۼ�ʱ�䳣��
	/* ����V=1/2*K*T^2,���Լ����һ�����ٶ� */
	DeltaV = 0.5f * K * Sumt * Sumt;
	/* �ڳ�ʼ�ٶȵĻ����������ٶ� */
	Speed.Form[0] = Speed.Vo + DeltaV;
	//�޷�
	if(Speed.Form[0] <= MIN_SPEED)//�Ե�ǰ��ʱ��Ƶ�����ܴﵽ������ٶ�
    Speed.Form[0] = MIN_SPEED;
  /***************************************************************************/
	/* ����S���ٶȱ� */	
	for(i=1;i<Speed.AccelTotalStep;i++){
	/* ����ʱ��������Ƶ�ʳɷ��ȵĹ�ϵ�����Լ����Ti,������ÿ�μ�����һ��ʱ�䣬���ڻ��۵���ǰʱ�� */
		Ti = 1.0f / Speed.Form[i-1];
		if(i < Speed.INC_AccelTotalStep)
		{
			/* �ۻ�ʱ�� */
			Sumt += Ti;
			/* �ٶȵı仯�� dV = 1/2 * K * Ti^2 */
			DeltaV = 0.5f * K * Sumt * Sumt;
			/* ���ݳ�ʼ�ٶȺͱ仯������ٶȱ� */
			Speed.Form[i] = Speed.Vo + DeltaV;
			/* Ϊ�˱�֤�����һ������ʹ��ʱ���Ͻ�����Ԥ�ڼ����ʱ��һ�£������һ�����д��� */
			if(i == Speed.INC_AccelTotalStep - 1){
				Sumt  =fabsf(Sumt - T);
			}
		}
		else{
			/* ʱ���ۻ� */
			Sumt += Ti;
			/* �����ٶ� */    
			temp = fabsf(T - Sumt);                                         
			DeltaV = 0.5f * K * temp * temp;
			Speed.Form[i] = Speed.Vt - DeltaV;
		}
	}
	return true;
}
/**
  * @brief  �ٶȾ���(�Ӽ���)
  * @param  pScurve��S���߽ṹ��ָ��
  * @retval ��
	*	@note 	���ж���ʹ�ã�ÿ��һ���жϣ�����һ��
  */
void Speed_Decision_S(void){
	/* ������� */
	static __IO uint8_t i = 0;
	static __IO uint32_t index = 0;
		/******************************************************************/
		/* ����Ϊһ���������� */
		i++;
		if(i == 2)
		{
			/* ������������������ */
			i = 0;
			/* �жϵ�ǰ��״̬ */
			switch(Stepper.status)
			{
				case ACCEL:
					if(Stepper.pos >=(Speed.AccelTotalStep-1))
					{
						Stepper.status = UNIFORM;
						index -= 1;
						break;
					}
					/* ��ȡÿһ���Ķ�ʱ������ֵ */
					Stepper.pluse_time = (uint16_t)(T1_FREQ / Speed.Form[index]/2);
					/* �������������� */
					index++;
					break;
				case DECEL:
					if(Stepper.pos >=(Speed.TotalStep-1))
					{
					/* ����ֹͣ״̬������ٶȱ��ҹر����ͨ�� */
						TMRA_PWM_OutputCmd(PWM_TIM,PWM_TIM_CH,DISABLE);
						TMRA_IntCmd(PWM_TIM,TMRA_FLAG_CMP_CH3,DISABLE);
						memset((void*)Speed.Form, 0, sizeof(float) * FORM_LEN);
						index = 0;
						Stepper.status = STOP;
						break;
					}
					/* ��ȡÿһ���Ķ�ʱ������ֵ */
					Stepper.pluse_time = (uint16_t)(T1_FREQ / Speed.Form[index]/2);
					/* ����λ�������ݼ� */
					index--;
					break;
				case UNIFORM:
					if(Stepper.pos >=Speed.DecPoint)
					{
						Stepper.status = DECEL;
					}
					break;
			}
			/* ����λ���������� */
			Stepper.pos++;
		}
		/* ��ȡ��ǰ��������ֵ */
		uint32_t tim_count=TMRA_GetCountValue(PWM_TIM);
		/* ������һ��ʱ�� */
		uint16_t tmp = tim_count + Stepper.pluse_time;
		/* ���ñȽ�ֵ */
		TMRA_SetCompareValue(PWM_TIM,PWM_TIM_CH,tmp);
}
/**
  * @brief  �������S���߼Ӽ���
  * @param  start_speed�������ٶȣ���λ��ת/����
	* @param  end_speed��Ŀ���ٶȣ���λ��ת/����
	* @param  acc_time������ʱ�䣬��λ����
	* @param  step���˶���������λ�������迼��ϸ�֣�
  * @retval true������
  * @retval false���������ô�����ٶȱ�ռ䲻��
	*	@note   ��
  */
bool Stepper_Move_S(int16_t start_speed, int16_t end_speed, float acc_time, int32_t step){
	if(Stepper.status != STOP)
		return false;
	/* ������� */
	if(Calcspeed_e(start_speed, end_speed, acc_time) != true)
		return false;
	if(step < 0)
	{
		step = -step;
		MOTOR_DIR_CCW();//��ʱ��
		dir_=-1;
	}
	else
	{
		MOTOR_DIR_CW();//˳ʱ��
		dir_=1;
	}
	/* ������ٵ㣬���˶�����С���������ٶ�ʱ�޷����s�Ӽ��� */
	if(step >= Speed.AccelTotalStep * 2)
	{
		Speed.TotalStep = step;
		Speed.DecPoint = Speed.TotalStep - Speed.AccelTotalStep;
	}
	else
	{
		ecode=2;
		//printf("�Ӽ��ٲ������ô���\r\n");
		return false;
	}
	/* ��ʼ���ṹ�� */
	memset(&Stepper, 0, sizeof(Stepper_Typedef));
	/* ��ʼ�����״̬ */
	Stepper.status = ACCEL;
	Stepper.pos = 0;
	/* �����һ���Ķ�ʱ������ */
	Stepper.pluse_time = (uint16_t)(T1_FREQ / Speed.Form[0]/2);
	/* ��������� */
	TMRA_SetCountValue(PWM_TIM,0);
	TMRA_SetCompareValue(PWM_TIM,PWM_TIM_CH,Stepper.pluse_time);
	if(dir_!=0){
		TMRA_PWM_OutputCmd(PWM_TIM,PWM_TIM_CH,ENABLE);
		TMRA_IntCmd(PWM_TIM, TMRA_FLAG_CMP_CH3,ENABLE);
	}
	cmd=1;
	return true;
}
/**
  * @brief  �ٶȾ���(����)
  * @param  *position �洢λ�ü����ĵ�ַ
  * @retval ��
	*	@note 	���ж���ʹ�ã�ÿ��һ���жϣ�����һ��
  */
void Speed_Decision_C(int32_t *position){
	if(cmd==1&&mode==0){
		n++;
		if(n>*position){//����ָ��λ��ͣ��
			TMRA_PWM_OutputCmd(PWM_TIM,PWM_TIM_CH,DISABLE);
			cmd=0;
		}
	}
}
/**
  * @brief  ����������ٶ����ƶ�
  * @param  speed���˶��ٶȣ���λ��ת/����
  * @param  *position ��ָ��λ����Ҫ��ʱ�жϴ����ĵ�ַ
	* @param  step���˶���������λ�������迼��ϸ�֣�
	*	@note   ��
  */
void Stepper_Move_C(int16_t speed,int32_t *position,int32_t step){	
	cmd=1;
	int32_t V=CONVER(speed);//���ٶȽ��е�λת����Hz��
	TMRA_SetPeriodValue(PWM_TIM,(uint32_t)(T1_FREQ/V/2));
	TMRA_SetCompareValue(PWM_TIM,PWM_TIM_CH,(uint32_t)(T1_FREQ/V/4));//����֮һռ�ձ�
	*position=(uint32_t)((float)step/(float)V*1000.0);//ת��Ϊ�ö�ʱ��Ƶ���»�������ж���
}

