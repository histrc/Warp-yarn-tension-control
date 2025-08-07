#ifndef __MP6602_H__
#define __MP6602_H__

#include "hc32_ll_gpio.h"
#include "hc32_ll_def.h"
#include "hc32_ll_spi.h"
#include "hc32_ll_utility.h"
#include "hc32_ll_fcg.h"

#define BSP_SPI_TIMEOUT                 (HCLK_VALUE)
// ����MP6602�ļĴ�����ַ
#define MP6602_CTRL_REG_ADDR       0x00  // CTRL�Ĵ�������ַ������ģʽѡ��
#define MP6602_CTRL_REG_WRITE_ADDR 0x01  // CTRL�Ĵ���д��ַ
#define MP6602_CTRL2_REG_ADDR       0x02  // CTRL2�Ĵ�������ַ ���綯��
#define MP6602_CTRL2_REG_WRITE_ADDR 0x03  // CTRL2�Ĵ���д��ַ
#define MP6602_ISET_REG_ADDR       0x04  // ISET�Ĵ�������ַ
#define MP6602_ISET_REG_WRITE_ADDR 0x05  // ISET�Ĵ���д��ַ
#define MP6602_STALL_REG_ADDR      0x06  // STALL�Ĵ�������ַ
#define MP6602_STALL_REG_WRITE_ADDR 0x07 // STALL�Ĵ���д��ַ
#define MP6602_BEMF_REG_ADDR      0x08  // BEMF�Ĵ�������ַ
#define MP6602_BEMF_REG_WRITE_ADDR 0x09 // BEMF�Ĵ���д��ַ
#define MP6602_TSTP_REG_ADDR      0x0A  // TSTP�Ĵ�������ַ
#define MP6602_TSTP_REG_WRITE_ADDR 0x0B // TSTP�Ĵ���д��ַ
#define MP6602_OCP_REG_ADDR      0x0C  // OCP�Ĵ�������ַ
#define MP6602_OCP_REG_WRITE_ADDR 0x0D // OCP�Ĵ���д��ַ
#define MP6602_FAULT_REG_ADDR      0x0E  // FAULT�Ĵ�������ַ
#define MP6602_FAULT_REG_WRITE_ADDR 0x0F // FAULT�Ĵ���д��ַ
// ���岽��ģʽ
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
