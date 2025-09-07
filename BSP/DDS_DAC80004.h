#ifndef __DDS_DAC80004_H__
#define __DDS_DAC80004_H__

#include "main.h"
#include "stdbool.h"
#include "DAC80004.h"

/**
 * @brief  正弦波生成结果结构体
 */
typedef struct {
    uint32_t points;              // 生成的点数
    double actual_freq;           // 实际输出频率 (Hz)
    double actual_sample_rate;    // 实际采样率 (Hz)
    double frequency_error;       // 频率误差 (Hz)
    double error_percent;         // 频率误差百分比 (%)
    bool success;                 // 生成是否成功 (false:失败, true:成功)
} SineWaveResult_t;

void DDS_Init(DAC80004_InitStruct *module);
bool DDS_Start_Precise(uint16_t *wave_data, uint16_t data_size, double sample_rate);
bool DDS_Start_Repeat(DAC80004_InitStruct *module,uint16_t *wave_data, uint16_t data_size, double sample_rate, uint32_t repeat_count);
void DDS_Stop(void);
void Generate_Smart_Sine_Wave(uint16_t *wave_buffer,
                              double target_freq, double max_sample_rate,
                              uint32_t min_points, uint32_t max_points,
                              uint16_t amplitude, uint16_t offset,
                              SineWaveResult_t *result);
void Encode_Wave(DAC80004_InitStruct *module, uint16_t *wave_buffer_in, 
                 uint16_t *wave_buffer_out, uint32_t points);
void Encode_Wave_DualDMA(DAC80004_InitStruct *module, uint16_t *wave_buffer_in, 
                        uint16_t *wave_buffer_high_out, uint16_t *wave_buffer_low_out, uint32_t points);

bool DDS_Start_DualDMA(DAC80004_InitStruct *module,uint16_t *wave_data_high,uint16_t *wave_data_low, 
                        uint16_t data_high_size,uint16_t data_low_size,
                        double sample_rate, uint32_t repeat_count);
void DDS_Stop_DualDMA(void);


#endif 
