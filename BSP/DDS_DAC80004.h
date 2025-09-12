#ifndef __DDS_DAC80004_H__
#define __DDS_DAC80004_H__


#include "main.h"
#include "stdbool.h"
#include "DAC80004.h"
#include "tim.h"

/**
 * @brief  波形模式枚举
 */
typedef enum {
    WAVE_MODE_NONE = 0,      // 无波形模式
    WAVE_MODE_SINE ,           // 正弦波模式
    WAVE_MODE_CV,                 // 循环伏安法模式
    WAVE_MODE_DPV               // 差分脉冲伏安法模式
} WaveMode_t;

/**
 * @brief  波形状态/阶段枚举
 */
typedef enum {
    WAVE_STAGE_STAGE1 = 0,            // 阶段1 - 第一个波形段
    WAVE_STAGE_STAGE2,            // 阶段2 - 第二个波形段  
    WAVE_STAGE_STAGE3,            // 阶段3 - 第三个波形段
    WAVE_STAGE_STAGE4            // 阶段4 - 第四个波形段
} WaveStage_t;
/**
 * @brief  DMA状态枚举
 */
typedef enum {
    DMA_STATE_IDLE = 0,           // 空闲状态
    DMA_STATE_RUNNING,            // 运行状态
    DMA_STATE_PAUSED,             // 暂停状态
    DMA_STATE_COMPLETED,          // 完成状态
    DMA_STATE_ERROR               // 错误状态
} DMAState_t;


typedef struct {
        uint16_t *data_ptr;       // 数据指针
        uint32_t data_points;     // 数据点数
        uint32_t current_count;   // 当前完成次数
        uint8_t transfer_complete; // 传输完成标志
}StreamData_t;

/**
 * @brief  DMA处理统一管理结构体
 */
typedef struct {
    // 基本状态
    WaveMode_t current_mode;      // 当前波形模式
    DMAState_t state;             // DMA状态    
    uint32_t repeat_count;      // DMA重复次数 (0=无限循环)
    StreamData_t stream5; // Stream5 (高16位) 管理
    StreamData_t stream4; // Stream4 (低16位) 管理
    WaveStage_t wave_stage;      // 当前波形阶段
} DMAHandler_t;



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

/**
 * @brief  循环伏安法结果结构体
 */
typedef struct {
    uint32_t points;                    // 总数据点数
    double actual_sample_rate;          // 实际采样率 (Hz)
    double total_duration;              // 总持续时间 (s)
    double actual_scan_rate;            // 实际扫描速率 (mV/s)
    double scan_rate_error;             // 扫描速率误差 (mV/s)
    double error_percent;               // 误差百分比 (%)
    uint32_t total_cycles;              // 实际循环次数
    // 起始区信息
    uint32_t initial_points;            // 起始区点数
    // 循环区信息
    uint32_t cycle_total_points;        // 所有循环的总点数
    // 终止区信息
    uint32_t final_points;              // 终止区点数
    
    bool success;                       // 生成是否成功
} CvWaveResult_t;






void SYNC_Cycle_Init(void);
void SYNC_Cycle_SetPara(const TIM_FreqConfig_t *config, double timer_clock, double spi_baudrate);
void SYNC_Cycle_Start(void);
void SYNC_Cycle_Stop(void);

void DDS_Init(DAC80004_InitStruct *module);
void Encode_Wave_DualDMA(DAC80004_InitStruct *module, uint16_t *wave_buffer_in, 
                        uint16_t *wave_buffer_high_out, uint16_t *wave_buffer_low_out, uint32_t points);

bool DDS_Start_DualDMA(DAC80004_InitStruct *module,WaveMode_t wavemode ,uint16_t *wave_data_high,
                        uint16_t *wave_data_low, uint16_t data_high_size,uint16_t data_low_size,
                        double sample_rate, uint32_t repeat_count);
void DDS_Stop_DualDMA(void);

void Generate_Smart_Sine_Wave(uint16_t *wave_buffer,
                              double target_freq, double max_sample_rate,
                              uint32_t min_points, uint32_t max_points,
                              uint16_t amplitude, uint16_t offset,
                              SineWaveResult_t *result);

bool Generate_And_Start_Sine_DDS(DAC80004_InitStruct *module,
                                 double target_freq, double max_sample_rate,
                                 uint32_t min_points, uint32_t max_points,
                                 uint16_t amplitude, uint16_t offset,
                                 uint32_t repeat_count,
                                 uint16_t *wave_buffer,
                                 uint16_t *wave_high_data,
                                 uint16_t *wave_low_data);

bool Generate_And_Start_CV_DDS(DAC80004_InitStruct *module,
                               double Initial_E, double Final_E,
                               double Scan_Limit1, double Scan_Limit2,
                               double Scan_Rate, double max_sample_rate,
                               uint32_t min_points, uint32_t max_points,
                               uint32_t repeat_count,
                               uint16_t *wave_buffer,
                               uint16_t *wave_high_data,
                               uint16_t *wave_low_data);

bool CV_DDS_Start(DAC80004_InitStruct *module,
                               double Initial_E, double Final_E,
                               double Scan_Limit1, double Scan_Limit2,
                               double Scan_Rate, double max_sample_rate,
                               uint32_t min_points, uint32_t max_points,
                               uint32_t repeat_count,
                               uint16_t *wave_high_data,
                               uint16_t *wave_low_data);


#endif 
