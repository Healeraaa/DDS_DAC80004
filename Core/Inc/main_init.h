#ifndef __MAIN_INIT_H
#define __MAIN_INIT_H

#include "main.h"
#include "Echem_stim.h"

extern dac80004_bus_t DAC80004_dev1;
extern DAC80004_InitStruct DAC80004_Module1;
extern PingPongConfig_t config;

extern uint16_t wave_high_data1[1024*8];
extern uint16_t wave_high_data2[1024*8];
extern uint16_t wave_low_data1[1024*8];
extern uint16_t wave_low_data2[1024*8];

// 函数声明
void Main_Variables_Init(void);


#endif
