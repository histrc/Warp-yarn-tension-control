/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
int32_t pos;
extern int32_t vn;//测试运动频率计数
int32_t main(void){
    /* Register write unprotected for some required peripherals. */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    //Clock Config
    App_ClkCfg();
    //Port Config
    App_PortCfg();
	//Int Config
	App_IntCfg();
    //TimerA Config
    App_TimerACfg();
    //SPIx Config
    App_SPIxCfg();
    /* Register write protected for some required peripherals. */
    LL_PERIPH_WP(LL_PERIPH_ALL);
	Stepper_start();//使能步进电机
	//MP6602_init();
	if(mode==0){Stepper_Move_C(190,&pos,(int32_t)SPR);}
//	if(mode==1){
//		for(int i=0;i<12;i++){
//			Stepper_Move_S(65, 650, 0.007f,(int32_t)(0.5*SPR));
//			while(Stepper.status != STOP);
//			//DDL_DelayMS(1000);
//			Stepper_Move_S(65, 650, 0.007f, -(int32_t)(0.5*SPR));
//			while(Stepper.status != STOP);
//		}
//	}
    for (;;) {
		if(mode==1){
		//for(int i=0;i<12;i++){
			Stepper_Move_S(65, 650, 0.008f,(int32_t)(0.5*SPR));
			while(Stepper.status != STOP);
			//DDL_DelayMS(1000);
			Stepper_Move_S(65, 650, 0.008f, (int32_t)(-0.5*SPR));
			while(Stepper.status != STOP);
			vn++;
		//}
	}
    }
}
