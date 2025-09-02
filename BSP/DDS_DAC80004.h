#ifndef __DDS_DAC80004_H__
#define __DDS_DAC80004_H__

#include "main.h"
#include "stdbool.h"
#include "DAC80004.h"

void DDS_Init(DAC80004_InitStruct *module);
bool DDS_Start_Precise(uint16_t *wave_data, uint16_t data_size, double sample_rate);
bool DDS_Start_Repeat(uint16_t *wave_data, uint16_t data_size, double sample_rate, uint32_t repeat_count);
void DDS_Stop(void);

#endif 
