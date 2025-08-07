#ifndef __MP6602_H__
#define __MP6602_H__

#include "hc32_ll_gpio.h"
#include "hc32_ll_def.h"
#include "hc32_ll_spi.h"
#include "hc32_ll_utility.h"
#include "hc32_ll_fcg.h"

#define BSP_SPI_TIMEOUT                 (HCLK_VALUE)
// 定义MP6602的寄存器地址
#define MP6602_CTRL_REG_ADDR       0x00  // CTRL寄存器读地址，步进模式选择
#define MP6602_CTRL_REG_WRITE_ADDR 0x01  // CTRL寄存器写地址
#define MP6602_CTRL2_REG_ADDR       0x02  // CTRL2寄存器读地址 反电动势
#define MP6602_CTRL2_REG_WRITE_ADDR 0x03  // CTRL2寄存器写地址
#define MP6602_ISET_REG_ADDR       0x04  // ISET寄存器读地址
#define MP6602_ISET_REG_WRITE_ADDR 0x05  // ISET寄存器写地址
#define MP6602_STALL_REG_ADDR      0x06  // STALL寄存器读地址
#define MP6602_STALL_REG_WRITE_ADDR 0x07 // STALL寄存器写地址
#define MP6602_BEMF_REG_ADDR      0x08  // BEMF寄存器读地址
#define MP6602_BEMF_REG_WRITE_ADDR 0x09 // BEMF寄存器写地址
#define MP6602_TSTP_REG_ADDR      0x0A  // TSTP寄存器读地址
#define MP6602_TSTP_REG_WRITE_ADDR 0x0B // TSTP寄存器写地址
#define MP6602_OCP_REG_ADDR      0x0C  // OCP寄存器读地址
#define MP6602_OCP_REG_WRITE_ADDR 0x0D // OCP寄存器写地址
#define MP6602_FAULT_REG_ADDR      0x0E  // FAULT寄存器读地址
#define MP6602_FAULT_REG_WRITE_ADDR 0x0F // FAULT寄存器写地址
// 定义步进模式
#define FULL_STEP        0x00
#define HALF_STEP        0x01   //1/2
#define QUARTER_STEP     0x02   //1/4
#define EIGHTH_STEP      0x03   //1/8
#define SIXTEENTH_STEP   0x04   //1/16
#define THIRTY_SECOND_STEP 0x05 //1/32

void App_SPIxCfg(void);
void MP6602_init(void);
uint16_t MP6602_ReadRegister(uint8_t address);
void MP6602_WriteRegister(uint8_t address, uint16_t data);
void MP6602_SetMicrostep(uint8_t mode);
uint8_t MP6602_ReadStepPosition(void);
uint16_t MP6602_ReadFaults(void);
#endif
