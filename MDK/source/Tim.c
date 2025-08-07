#include "Tim.h"
#include "hc32_ll.h"
#include "control_step.h"
extern int32_t n;//定速计数
extern int32_t ns;//s形计数
extern int32_t pos;
int32_t vn=0;
float vs=0;//运动频率
//Clock Config
/* INT_SRC_TMRA_3_OVF Callback. */
static void INT_SRC_TMRA_1_OVF_IrqCallback(void);
/* INT_SRC_TMRA_3_OVF Callback. */
static void INT_SRC_TMRA_3_OVF_IrqCallback(void);
/* INT_SRC_TMRA_3_CMP Callback. */
static void INT_SRC_TMRA_3_CMP_IrqCallback(void);
//Clock Config
void App_ClkCfg(void)
{
    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                   CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));
    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAM_ALL, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    /* flash read wait cycle setting */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* XTAL config */
    stc_clock_xtal_init_t stcXtalInit;
    (void)CLK_XtalStructInit(&stcXtalInit);
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_HIGH;
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);
    /* MPLL config */
    stc_clock_pll_init_t stcMPLLInit;
    (void)CLK_PLLStructInit(&stcMPLLInit);
    stcMPLLInit.PLLCFGR = 0UL;
    stcMPLLInit.PLLCFGR_f.PLLM = (1UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLN = (50UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLP = (2UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLQ = (2UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLR = (2UL - 1UL);
    stcMPLLInit.u8PLLState = CLK_PLL_ON;
    stcMPLLInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcMPLLInit);
    /* 3 cycles for 126MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    /* Switch driver ability */
    PWC_HighSpeedToHighPerformance();
    /* Set the system clock source */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
}
//Int Config
void App_IntCfg(void)
{
    stc_irq_signin_config_t stcIrq;

    /* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_TMRA_1_OVF;
    stcIrq.enIRQn = INT080_IRQn;
    stcIrq.pfnCallback = &INT_SRC_TMRA_1_OVF_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT080_IRQn);
    NVIC_SetPriority(INT080_IRQn, DDL_IRQ_PRIO_15);
    NVIC_EnableIRQ(INT080_IRQn);
	 /* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_TMRA_3_CMP;
    stcIrq.enIRQn = INT081_IRQn;
    stcIrq.pfnCallback = &INT_SRC_TMRA_3_CMP_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT081_IRQn);
    NVIC_SetPriority(INT081_IRQn, DDL_IRQ_PRIO_10);
    NVIC_EnableIRQ(INT081_IRQn);
	
	/* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_TMRA_3_OVF;
    stcIrq.enIRQn = INT082_IRQn;
    stcIrq.pfnCallback = &INT_SRC_TMRA_3_OVF_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT082_IRQn);
    NVIC_SetPriority(INT082_IRQn, DDL_IRQ_PRIO_15);
    NVIC_EnableIRQ(INT082_IRQn);

}
void App_TimerACfg(void)
{
    stc_tmra_init_t stcTmraInit;
	stc_tmra_pwm_init_t stcPwmInit;
	/* Enable TMRA_1 peripheral clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_1, ENABLE);

    /************************* Configure TMRA_1 counter *************************/
    (void)TMRA_StructInit(&stcTmraInit);
    /* Config software count */
    stcTmraInit.sw_count.u8ClockDiv = TMRA_CLK_DIV32;//32计数，256加减速测试
    stcTmraInit.sw_count.u8CountMode = TMRA_MD_SAWTOOTH;
    stcTmraInit.sw_count.u8CountDir = TMRA_DIR_UP;
    stcTmraInit.u32PeriodValue = 0xC35U;//测加减速0x9896U;//C35U 1ms进一次中断
    (void)TMRA_Init(CM_TMRA_1, &stcTmraInit);
	/* Enable overflow interrupt */
	TMRA_IntCmd(CM_TMRA_1, TMRA_INT_OVF, ENABLE);
    /* Start timerA_1 */
    TMRA_Start(CM_TMRA_1);
	
	/* Enable TMRA_3 peripheral clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_3, ENABLE);
    /************************* Configure TMRA_3 counter *************************/
    (void)TMRA_StructInit(&stcTmraInit);
    /* Config software count */
    stcTmraInit.sw_count.u8ClockDiv = TMRA_CLK_DIV1;
    stcTmraInit.sw_count.u8CountMode = TMRA_MD_SAWTOOTH;
    stcTmraInit.sw_count.u8CountDir = TMRA_DIR_UP;
    stcTmraInit.u32PeriodValue = 0xFFFF;//20000U-1U;888
    (void)TMRA_Init(PWM_TIM, &stcTmraInit);
	 /* Enable overflow interrupt */
    TMRA_IntCmd(PWM_TIM, TMRA_INT_OVF, ENABLE);
	/************************* Configure TMRA_3_3 CMP ***************************/
	(void)TMRA_PWM_StructInit(&stcPwmInit);
	stcPwmInit.u32CompareValue = 0x0U;//10000U
    stcPwmInit.u16StartPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_INVT;
	stcPwmInit.u16PeriodMatchPolarity=TMRA_PWM_HOLD;
	(void)TMRA_PWM_Init(PWM_TIM, PWM_TIM_CH, &stcPwmInit);
//	/* PWM pin function set */
	TMRA_SetFunc(PWM_TIM, PWM_TIM_CH, TMRA_FUNC_CMP);
	/* PWM output command */
	TMRA_PWM_OutputCmd(PWM_TIM,PWM_TIM_CH,ENABLE);
	/* Enable CMP interrupt */
    TMRA_IntCmd(PWM_TIM, TMRA_INT_CMP_CH3, ENABLE);
	/* Start timerA_3 */
    TMRA_Start(PWM_TIM); 
	
}
/* INT_SRC_TMRA_1_OVF Callback. *///1ms
static void INT_SRC_TMRA_1_OVF_IrqCallback(void)
{
	static uint32_t interrupt_count = 0;
	if(mode==0)Speed_Decision_C(&pos);
	interrupt_count++;
	if(interrupt_count==1000){
		vs=vn;
		vn=0;
		interrupt_count=0;
	}
//	
}
/* INT_SRC_TMRA_3_OVF Callback. */
static void INT_SRC_TMRA_3_OVF_IrqCallback(void)
{

}
/* INT_SRC_TMRA_3_CMP Callback. */
static void INT_SRC_TMRA_3_CMP_IrqCallback(void)
{
	TMRA_ClearStatus(PWM_TIM,TMRA_FLAG_CMP_CH3);//清除定时器中断
	ns++;
	if(mode==1){Speed_Decision_S();}
}
