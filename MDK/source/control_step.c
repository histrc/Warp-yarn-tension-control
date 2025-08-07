#include "control_step.h"
#include <math.h>
#include <string.h>
#define FALSE 0
#define TRUE 1
/* 算法相关结构体变量定义 */
uint8_t ecode=0;//1,缓存长度不足。2，加减速参数错误
int32_t n=0;//定速计数
int32_t ns=0;//s形计数
int8_t dir_=0; //（指示转动方向，不用来定义）1表示顺时针，-1表示逆时针
SpeedCalc_TypeDef Speed;
Stepper_Typedef Stepper = {0};
uint8_t cmd;//控制状态标志位
//Port Config
#define STEP_EN_PORT 		GPIO_PORT_C //步进电机EN引脚定义
#define STEP_EN_PIN 		GPIO_PIN_07
#define STEP_DIR_PORT 		GPIO_PORT_C//步进电机DIR引脚定义
#define STEP_DIR_PIN 		GPIO_PIN_06
#define LED_PORT 			GPIO_PORT_B // 板载LED灯
#define LED_PIN 			GPIO_PIN_12
void App_PortCfg(void)
{
    /* GPIO initialize */
    stc_gpio_init_t stcGpioInit;
    /* PC7 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(STEP_EN_PORT,STEP_EN_PIN, &stcGpioInit);//EN引脚定义

    /* PC6 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(STEP_DIR_PORT,STEP_DIR_PIN, &stcGpioInit);//DIR引脚定义

    /* PB12 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(LED_PORT, LED_PIN, &stcGpioInit);//LED引脚定义

    GPIO_SetFunc(GPIO_PORT_C,GPIO_PIN_00,GPIO_FUNC_42);//SPI1-SS0
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_00,GPIO_FUNC_43);//SPI1-SCK
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_02,GPIO_FUNC_40);//SPI1-MOSI
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_03,GPIO_FUNC_41);//SPI1-MISO
    
    GPIO_SetFunc(GPIO_PORT_C,GPIO_PIN_08,GPIO_FUNC_4);//TIMA-3-PWM3
    
}
/**
  * @brief  使能步进电机
	*	@note 	无
  */
void Stepper_start(void){
	GPIO_SetPins(STEP_EN_PORT,STEP_EN_PIN);//ENABLE4
}
/**
  * @brief  关闭步进电机使能
	*	@note 	无
  */
void Stepper_stop(void){
	GPIO_ResetPins(STEP_EN_PORT,STEP_EN_PIN);//ENABLE4
}
/**
  * @brief  指数型曲线速度表计算 
  * @param  Vo：初速度
  * @param  Vt：末速度
  * @param  T_s：加速时间
  * @retval 0：正常，1：参数错误
	*	@note 	无
  */
static bool Calcspeed_e(int32_t Vo_rpm, int32_t Vt_rpm, float T_s)
{
    /* ------------------ 1. 参数转换 ------------------ */
    float Vo = CONVER(Vo_rpm);          // 步/秒
    float Vt = CONVER(Vt_rpm);
    float T  = T_s * 1000.0f;           // 转成 ms，后面按 1ms 节拍跑
    float tau = T / 3.0f;               // 指数时间常数，可调

    /* ------------------ 2. 计算总步数 ------------------ */
    float total_dist = (Vo + Vt) * 0.5f * T * 0.001f;   // 梯形面积近似
    Speed.AccelTotalStep = (int32_t)(total_dist + 0.5f);
    if (Speed.AccelTotalStep % 2) Speed.AccelTotalStep++;
    if (FORM_LEN < Speed.AccelTotalStep) { ecode = 1; return false; }
    /* ------------------ 3. 生成指数速度表 --------------- */
    for (int32_t i = 0; i < Speed.AccelTotalStep; i++) {
        float t = (float)i * T / Speed.AccelTotalStep;   // 当前相对时间
        float v = Vo + (Vt - Vo) * (1.0f - expf(-t / tau));
        Speed.Form[i] = v;
        if (Speed.Form[i] < MIN_SPEED) Speed.Form[i] = MIN_SPEED;
    }

    /* 减速表与加速表完全对称，直接反向遍历即可 */
    Speed.INC_AccelTotalStep = Speed.AccelTotalStep / 2;
    Speed.Dec_AccelTotalStep = Speed.AccelTotalStep - Speed.INC_AccelTotalStep;
    return true;
}
static bool Calcspeed_s(int32_t Vo,int32_t Vt,float T){
	int32_t i = 0;
	float Vm =0.0f;                // 中间点速度
	float K = 0.0f;                // 加加速度
	float Ti = 0.0f;               // 时间间隔 dt
	float Sumt = 0.0f;             // 时间累加量
	float DeltaV = 0.0f;           // 速度的增量dv
	float temp = 0.0f;             // 中间变量
	/***************************************************************************/
	/* 计算初、末速度 */
	Speed.Vo = CONVER(Vo);
	Speed.Vt = CONVER(Vt);
	/***************************************************************************/
	/* 计算初始参数 */
	T = T / 2.0f;						//加加速段的时间（加速度斜率>0的时间）
	Vm = (Speed.Vo + Speed.Vt) / 2.0f;	//计算中点的速度
	K = fabsf((2.0f * (Vm - Speed.Vo)) / (T * T));// 根据中点速度计算加加速度	
	Speed.INC_AccelTotalStep = (int32_t)((Speed.Vo * T) + ((K * T * T* T) / 6.0f));// 加加速需要的步数(int32_t)((Speed.Vo * T) + ((K * T * T* T) / 6.0f));
	Speed.Dec_AccelTotalStep = (int32_t)(((Speed.Vt + Speed.Vo) * T - Speed.INC_AccelTotalStep));// 减加速需要的步数 S = Vt * Time - S1  (int32_t)(((Speed.Vt + Speed.Vo) * T - Speed.INC_AccelTotalStep))
	/***************************************************************************/
	/* 计算共需要的步数，并校检内存大小，申请内存空间存放速度表 */
	Speed.AccelTotalStep = Speed.Dec_AccelTotalStep + Speed.INC_AccelTotalStep;              // 加速需要的步数 
	if(Speed.AccelTotalStep  % 2 != 0)     // 由于浮点型数据转换成整形数据带来了误差,所以这里加1
		Speed.AccelTotalStep  += 1;
	/* 判断内存长度 */
	if(FORM_LEN < Speed.AccelTotalStep)
	{
		ecode=1;
		//printf("FORM_LEN 缓存长度不足\r\n，请将 FORM_LEN 修改为 %d \r\n", Speed.AccelTotalStep);
		return false;
	}
	/***************************************************************************/
	/* 计算第一步的时间 */
	/* 根据第一步的时间计算，第一步的速度和脉冲时间间隔 */
	/* 根据位移为0的时候的情况，计算时间的关系式 -> 根据位移和时间的公式S = 1/2 * K * Ti^3  可得 Ti=6 * 1 / K开1/3次方 */
	Ti = pow(6.0f * 1.0f / K, 1.0f / 3.0f); //开方求解 Ti 时间常数
	Sumt += Ti;//累计时间常数
	/* 根据V=1/2*K*T^2,可以计算第一步的速度 */
	DeltaV = 0.5f * K * Sumt * Sumt;
	/* 在初始速度的基础上增加速度 */
	Speed.Form[0] = Speed.Vo + DeltaV;
	//限幅
	if(Speed.Form[0] <= MIN_SPEED)//以当前定时器频率所能达到的最低速度
    Speed.Form[0] = MIN_SPEED;
  /***************************************************************************/
	/* 计算S形速度表 */	
	for(i=1;i<Speed.AccelTotalStep;i++){
	/* 根据时间周期与频率成反比的关系，可以计算出Ti,在这里每次计算上一步时间，用于积累到当前时间 */
		Ti = 1.0f / Speed.Form[i-1];
		if(i < Speed.INC_AccelTotalStep)
		{
			/* 累积时间 */
			Sumt += Ti;
			/* 速度的变化量 dV = 1/2 * K * Ti^2 */
			DeltaV = 0.5f * K * Sumt * Sumt;
			/* 根据初始速度和变化量求得速度表 */
			Speed.Form[i] = Speed.Vo + DeltaV;
			/* 为了保证在最后一步可以使得时间严谨的与预期计算的时间一致，在最后一步进行处理 */
			if(i == Speed.INC_AccelTotalStep - 1){
				Sumt  =fabsf(Sumt - T);
			}
		}
		else{
			/* 时间累积 */
			Sumt += Ti;
			/* 计算速度 */    
			temp = fabsf(T - Sumt);                                         
			DeltaV = 0.5f * K * temp * temp;
			Speed.Form[i] = Speed.Vt - DeltaV;
		}
	}
	return true;
}
/**
  * @brief  速度决策(加减速)
  * @param  pScurve：S曲线结构体指针
  * @retval 无
	*	@note 	在中断中使用，每进一次中断，决策一次
  */
void Speed_Decision_S(void){
	/* 脉冲计数 */
	static __IO uint8_t i = 0;
	static __IO uint32_t index = 0;
		/******************************************************************/
		/* 两次为一个脉冲周期 */
		i++;
		if(i == 2)
		{
			/* 脉冲周期完整后清零 */
			i = 0;
			/* 判断当前的状态 */
			switch(Stepper.status)
			{
				case ACCEL:
					if(Stepper.pos >=(Speed.AccelTotalStep-1))
					{
						Stepper.status = UNIFORM;
						index -= 1;
						break;
					}
					/* 获取每一步的定时器计数值 */
					Stepper.pluse_time = (uint16_t)(T1_FREQ / Speed.Form[index]/2);
					/* 步数置索引递增 */
					index++;
					break;
				case DECEL:
					if(Stepper.pos >=(Speed.TotalStep-1))
					{
					/* 进入停止状态，清空速度表并且关闭输出通道 */
						TMRA_PWM_OutputCmd(PWM_TIM,PWM_TIM_CH,DISABLE);
						TMRA_IntCmd(PWM_TIM,TMRA_FLAG_CMP_CH3,DISABLE);
						memset((void*)Speed.Form, 0, sizeof(float) * FORM_LEN);
						index = 0;
						Stepper.status = STOP;
						break;
					}
					/* 获取每一步的定时器计数值 */
					Stepper.pluse_time = (uint16_t)(T1_FREQ / Speed.Form[index]/2);
					/* 步数位置索引递减 */
					index--;
					break;
				case UNIFORM:
					if(Stepper.pos >=Speed.DecPoint)
					{
						Stepper.status = DECEL;
					}
					break;
			}
			/* 步数位置索引递增 */
			Stepper.pos++;
		}
		/* 获取当前计数器数值 */
		uint32_t tim_count=TMRA_GetCountValue(PWM_TIM);
		/* 计算下一次时间 */
		uint16_t tmp = tim_count + Stepper.pluse_time;
		/* 设置比较值 */
		TMRA_SetCompareValue(PWM_TIM,PWM_TIM_CH,tmp);
}
/**
  * @brief  步进电机S曲线加减速
  * @param  start_speed：启动速度，单位：转/分钟
	* @param  end_speed：目标速度，单位：转/分钟
	* @param  acc_time：加速时间，单位：秒
	* @param  step：运动步数，单位：步（需考虑细分）
  * @retval true：正常
  * @retval false：参数设置错误或速度表空间不足
	*	@note   无
  */
bool Stepper_Move_S(int16_t start_speed, int16_t end_speed, float acc_time, int32_t step){
	if(Stepper.status != STOP)
		return false;
	/* 计算参数 */
	if(Calcspeed_e(start_speed, end_speed, acc_time) != true)
		return false;
	if(step < 0)
	{
		step = -step;
		MOTOR_DIR_CCW();//逆时针
		dir_=-1;
	}
	else
	{
		MOTOR_DIR_CW();//顺时针
		dir_=1;
	}
	/* 计算减速点，当运动步数小于两倍加速段时无法完成s加减速 */
	if(step >= Speed.AccelTotalStep * 2)
	{
		Speed.TotalStep = step;
		Speed.DecPoint = Speed.TotalStep - Speed.AccelTotalStep;
	}
	else
	{
		ecode=2;
		//printf("加减速参数设置错误！\r\n");
		return false;
	}
	/* 初始化结构体 */
	memset(&Stepper, 0, sizeof(Stepper_Typedef));
	/* 初始化电机状态 */
	Stepper.status = ACCEL;
	Stepper.pos = 0;
	/* 计算第一步的定时器参数 */
	Stepper.pluse_time = (uint16_t)(T1_FREQ / Speed.Form[0]/2);
	/* 清零计数器 */
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
  * @brief  速度决策(定速)
  * @param  *position 存储位置计数的地址
  * @retval 无
	*	@note 	在中断中使用，每进一次中断，决策一次
  */
void Speed_Decision_C(int32_t *position){
	if(cmd==1&&mode==0){
		n++;
		if(n>*position){//到达指定位置停下
			TMRA_PWM_OutputCmd(PWM_TIM,PWM_TIM_CH,DISABLE);
			cmd=0;
		}
	}
}
/**
  * @brief  步进电机恒速定长移动
  * @param  speed：运动速度，单位：转/分钟
  * @param  *position 到指定位置需要定时中断次数的地址
	* @param  step：运动步数，单位：步（需考虑细分）
	*	@note   无
  */
void Stepper_Move_C(int16_t speed,int32_t *position,int32_t step){	
	cmd=1;
	int32_t V=CONVER(speed);//对速度进行单位转换（Hz）
	TMRA_SetPeriodValue(PWM_TIM,(uint32_t)(T1_FREQ/V/2));
	TMRA_SetCompareValue(PWM_TIM,PWM_TIM_CH,(uint32_t)(T1_FREQ/V/4));//二分之一占空比
	*position=(uint32_t)((float)step/(float)V*1000.0);//转换为该定时器频率下会产生的中断数
}

