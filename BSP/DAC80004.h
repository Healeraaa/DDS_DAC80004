#ifndef __DAC80004_H__
#define __DAC80004_H__

#include "main.h"

#define DAC_W    0x0   //写操作
#define DAC_R    0x1   //读操作

#define DAC_CMD_WRITE_BUFF_N      0x0   // 写入n通道缓冲区（D27-D24=0000）
#define DAC_CMD_UPDATE_BUFF_N     0x1   // 更新n通道缓冲区到输出（D27-D24=0001）
#define DAC_CMD_WRITE_UPDATE_ALL  0x2   // 写缓冲区并更新所有通道（软件LDAC，D27-D24=0010）
#define DAC_CMD_WRITE_UPDATE_N    0x3   // 写缓冲区并更新指定通道（D27-D24=0011）
#define DAC_CMD_CLEAR_MODE        0x7   // 软件复位（D27-D24=0101）

#define DAC_CH_A          0x0     // 通道A（D23-D20=0000）
#define DAC_CH_B          0x1     // 通道B（D23-D20=0001）
#define DAC_CH_C          0x2     // 通道C（D23-D20=0010）
#define DAC_CH_D          0x3     // 通道D（D23-D20=0011）
#define DAC_CH_ALL        0xF     // 所有通道（D23-D20=1111）

#define DAC_MODE_NO         0x0     // 无操作（D3-D0 = 0000）



typedef struct
{
	GPIO_TypeDef * LDAC_PORT;
	GPIO_TypeDef * CLR_PORT;
    GPIO_TypeDef * CS_PORT;
	uint32_t LDAC_PIN;
	uint32_t CLR_PIN;
	uint32_t CS_PIN;
}dac80004_bus_t;

typedef struct
{
    dac80004_bus_t *dev;
    uint8_t WR_Buff;
    uint8_t Command_Buff;
    uint8_t Channel_Buff;
    uint16_t Data_Buff;
    uint8_t Mode_Buff;
    uint32_t TX_Data;
}DAC80004_InitStruct;

void DAC80004_WR_Config(DAC80004_InitStruct *module, uint8_t WR);
void DAC80004_Command_Config(DAC80004_InitStruct *module, uint8_t Command);
void DAC80004_Channel_Config(DAC80004_InitStruct *module, uint8_t Channel   );
void DAC80004_Data_Set(DAC80004_InitStruct *module, uint16_t Data);
void DAC80004_Mode_Config(DAC80004_InitStruct *module, uint8_t Mode);

void DAC80004_Init(DAC80004_InitStruct *module);
void DAC80004_WriteData(DAC80004_InitStruct *module);


#endif 
