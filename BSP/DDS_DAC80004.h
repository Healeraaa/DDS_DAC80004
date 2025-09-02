#ifndef __DDS_DAC80004_H__
#define __DDS_DAC80004_H__

#include "main.h"
#include "DAC80004.h"

void DDS_Init(DAC80004_InitStruct *module);
void DDS_Start(uint16_t *wave_data, uint16_t data_size, uint32_t sample_rate);
void DDS_Stop(void);

#endif 
