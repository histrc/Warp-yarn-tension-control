#include "mp6602.h"
#define BSP_SPI_CS_ACTIVE()             GPIO_ResetPins(GPIO_PORT_C,GPIO_PIN_00)
#define BSP_SPI_CS_INACTIVE()           GPIO_SetPins(GPIO_PORT_C,GPIO_PIN_00)
//SPIx Config

void App_SPIxCfg(void)
{
    stc_spi_init_t stcSpiInit;
    stc_spi_delay_t stcSpiDelay;

    /* Enable SPI1 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_SPI1, ENABLE);
    /************************* Configure SPI1***************************/
    SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode = SPI_4_WIRE;
    stcSpiInit.u32TransMode = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave = SPI_MASTER;
    stcSpiInit.u32Parity = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode = SPI_MD_3;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV256;
    stcSpiInit.u32DataBits = SPI_DATA_SIZE_16BIT;
    stcSpiInit.u32FirstBit = SPI_FIRST_MSB;
    stcSpiInit.u32SuspendMode = SPI_COM_SUSP_FUNC_OFF;
    stcSpiInit.u32FrameLevel = SPI_1_FRAME;
    (void)SPI_Init(CM_SPI1, &stcSpiInit);

    SPI_DelayStructInit(&stcSpiDelay);
    stcSpiDelay.u32IntervalDelay = SPI_INTERVAL_TIME_1SCK;
    stcSpiDelay.u32ReleaseDelay = SPI_RELEASE_TIME_1SCK;
    stcSpiDelay.u32SetupDelay = SPI_SETUP_TIME_1SCK;
    (void)SPI_DelayTimeConfig(CM_SPI1, &stcSpiDelay);

    /* SPI loopback function configuration */
    SPI_SetLoopbackMode(CM_SPI1, SPI_LOOPBACK_INVD);
    /* SPI parity check error self diagnosis configuration */
    SPI_ParityCheckCmd(CM_SPI1, DISABLE);
    /* SPI valid SS signal configuration */
    SPI_SSPinSelect(CM_SPI1, SPI_PIN_SS0);
    /* SPI SS signal valid level configuration */
    SPI_SetSSValidLevel(CM_SPI1, SPI_PIN_SS0, SPI_SS_VALID_LVL_LOW);
    /* Enable SPI1 */
    SPI_Cmd(CM_SPI1, ENABLE);
}

void MP6602_init(void){
	BSP_SPI_CS_INACTIVE();
}
uint16_t MP6602_ReadRegister(uint8_t address) {
    uint16_t command = (address << 12) & 0xF000; // 地址左移12位，R/W=0
    uint16_t rx_data;
    
    BSP_SPI_CS_ACTIVE(); // 拉低nSCS
    DDL_DelayUS(1);                // 确保nSCS设置时间（t4=30ns）
	SPI_Trans(CM_SPI1,&command,16,HCLK_VALUE);
    SPI_Receive(CM_SPI1,&rx_data,12,HCLK_VALUE); // 发送16位命令，接收数据
    BSP_SPI_CS_INACTIVE(); // 拉高nSCS
    DDL_DelayUS(1);                // 确保nSCS非激活时间（t10=100ns）
    
    return rx_data & 0x0FFF; // 返回12位数据
}

void MP6602_WriteRegister(uint8_t address, uint16_t data) {
    uint16_t command = ((address << 12) & 0xF000) | 0x1000 | (data & 0x0FFF); // 地址+R/W=1+数据
    
    BSP_SPI_CS_ACTIVE(); // 拉低nSCS
    DDL_DelayUS(1);                // 确保nSCS设置时间
    SPI_Trans(CM_SPI1,&command,16,HCLK_VALUE); // 发送16位命令，忽略接收数据
    BSP_SPI_CS_INACTIVE(); // 拉高nSCS
    DDL_DelayUS(1);               // 确保nSCS非激活时间
}
void MP6602_SetMicrostep(uint8_t mode) {
    uint16_t ctrl=0x100;
	uint16_t temp;
    uint8_t ms_bits=0x00;
    ms_bits=mode;
	ctrl=ctrl&0xFFC7;
	temp=ms_bits<<6;
	ctrl=ctrl|temp;
    MP6602_WriteRegister(MP6602_CTRL_REG_WRITE_ADDR,ctrl);
}
//void MP6602_SetStallDetection(MP6602_t *dev, uint8_t threshold, uint8_t cycles) {
//    // 阈值是 8 位 (STH7-STH0)，周期是 4 位 (STD3-STD0)
//    if (threshold > 0xFF) threshold = 0xFF;
//    if (cycles > 0x0F) cycles = 0x0F;
//    uint16_t stall = (threshold << 4) | (cycles & 0x0F);
//    MP6602_WriteRegister(dev, MP6602_STALL_WRITE, stall);
//}
uint8_t MP6602_ReadStepPosition(void) {
    uint16_t tstp = MP6602_ReadRegister(MP6602_TSTP_REG_ADDR);
    return tstp & 0x7F; // STP6-STP0，7 位
}

uint16_t MP6602_ReadFaults(void) {
    return MP6602_ReadRegister(MP6602_FAULT_REG_ADDR);
}
