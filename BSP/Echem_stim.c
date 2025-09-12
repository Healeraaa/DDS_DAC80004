#include "Echem_stim.h"
#include "DDS_DAC80004.h"  // 引用基础DDS功能
#include "spi.h"
#include "tim.h"
#include "dma.h"
#include <math.h>       

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 全局乒乓DMA管理结构体
volatile PingPongDMA_t g_pingpong_dma = {0};
EchemResult_t g_echem_result = {0};

// 添加全局变量保存参数（供CV_Fill_Next_Buffer使用）
static DAC80004_InitStruct *g_dac_module = NULL;
static EchemCV_Params_t g_cv_params = {0};


// 回调函数指针
static EchemProgressCallback_t g_progress_callback = NULL;
static EchemErrorCallback_t g_error_callback = NULL;

// 静态缓冲区指针（供中断使用）
static uint16_t *g_cv_wave_high_data1 = NULL;
static uint16_t *g_cv_wave_high_data2 = NULL;
static uint16_t *g_cv_wave_low_data1 = NULL;
static uint16_t *g_cv_wave_low_data2 = NULL;




// ==================== 通用乒乓DMA管理函数 ====================

/**
 * @brief  停止乒乓DMA传输（通用函数，所有实验方法都可使用）
 * @retval None
 */
void PingPong_DMA_Stop(void)
{
    // 停止DMA流
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
    
    // 等待DMA停止
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5) || LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4)) {}
    
    // 停止定时器
    LL_TIM_DisableCounter(TIM1);
    
    // 禁用DMA请求
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_TIM_DisableDMAReq_UPDATE(TIM1);
    LL_TIM_DisableDMAReq_CC4(TIM1);
    
    // 停止同步信号
    SYNC_Cycle_Stop();
    
    // 禁用DMA中断
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_4);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_4);
    
    // 清除DMA标志位
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_TE5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);
    LL_DMA_ClearFlag_DME5(DMA2);
    LL_DMA_ClearFlag_FE5(DMA2);
    
    LL_DMA_ClearFlag_TC4(DMA2);
    LL_DMA_ClearFlag_TE4(DMA2);
    LL_DMA_ClearFlag_HT4(DMA2);
    LL_DMA_ClearFlag_DME4(DMA2);
    LL_DMA_ClearFlag_FE4(DMA2);
    
    // 更新状态
    g_pingpong_dma.is_running = false;
    g_pingpong_dma.need_fill_buffer = false;
}

/**
 * @brief  暂停乒乓DMA传输
 * @retval None
 */
void PingPong_DMA_Pause(void)
{
    if (!g_pingpong_dma.is_running) {
        return;
    }
    
    // 停止定时器（暂停数据发送）
    LL_TIM_DisableCounter(TIM1);
    
    // 更新状态
    g_echem_result.state = ECHEM_STATE_PAUSED;
}

/**
 * @brief  恢复乒乓DMA传输
 * @retval None
 */
void PingPong_DMA_Resume(void)
{
    if (!g_pingpong_dma.is_running || g_pingpong_dma.transfer_complete) {
        return;
    }
    
    // 恢复定时器（继续数据发送）
    LL_TIM_EnableCounter(TIM1);
    
    // 更新状态
    g_echem_result.state = ECHEM_STATE_RUNNING;
}

/**
 * @brief  检查乒乓DMA传输状态
 * @retval true: 传输完成, false: 传输进行中
 */
bool PingPong_DMA_IsComplete(void)
{
    return g_pingpong_dma.transfer_complete;
}

/**
 * @brief  获取乒乓DMA传输进度
 * @retval 传输进度百分比 (0-100)
 */
uint8_t PingPong_DMA_GetProgress(void)
{
    if (g_pingpong_dma.total_points == 0) {
        return 0;
    }
    
    uint32_t progress = (g_pingpong_dma.points_sent * 100) / g_pingpong_dma.total_points;
    return (progress > 100) ? 100 : (uint8_t)progress;
}

/**
 * @brief  重置乒乓DMA状态
 * @retval None
 */
void PingPong_DMA_Reset(void)
{
    g_pingpong_dma.active_buffer = 0;
    g_pingpong_dma.total_points = 0;
    g_pingpong_dma.points_sent = 0;
    g_pingpong_dma.buffer_size = 0;
    g_pingpong_dma.transfer_complete = false;
    g_pingpong_dma.is_running = false;
    g_pingpong_dma.need_fill_buffer = false;
    g_pingpong_dma.buffer_to_fill = 0;
}

// ==================== CV专用函数 ====================



/**
 * @brief  生成CV波形数据到指定缓冲区
 * @param  module: DAC80004模块结构体指针
 * @param  wave_high_data: 高16位数据缓冲区
 * @param  wave_low_data: 低16位数据缓冲区
 * @param  start_point: 起始点索引
 * @param  points_to_fill: 要填充的点数
 * @param  cv_params: CV参数结构体指针
 * @param  initial_points: 起始段点数
 * @param  cycle_points: 循环段点数
 * @param  final_points: 结束段点数
 * @retval None
 */
static void Generate_CV_Data_Partial(DAC80004_InitStruct *module,
                                    uint16_t *wave_high_data, uint16_t *wave_low_data,
                                    uint32_t start_point, uint32_t points_to_fill,
                                    const EchemCV_Params_t *cv_params)
{
    uint32_t control_mask = module->TX_Data & ~(0xFFFF << 4);
    uint32_t encoded_data;
    uint32_t cycles = (cv_params->cycles > 0) ? cv_params->cycles : 1;
    
    // 直接使用结构体中的点数
    uint32_t initial_points = cv_params->initial_points;
    uint32_t cycle_points = cv_params->cycle_points;
    uint32_t final_points = cv_params->final_points;
    uint32_t total_cycle_points = cycle_points * cycles;  // ✅ 所有循环的总点数

    for (uint32_t i = 0; i < points_to_fill; i++) {
        uint32_t global_index = start_point + i;
        double voltage = 0.0;

        if (global_index < initial_points) {
            // 起始阶段：Initial_E → Scan_Limit1
            double progress = (initial_points > 1) ? (double)global_index / (initial_points - 1) : 0.0;
            voltage = cv_params->Initial_E + progress * (cv_params->Scan_Limit1 - cv_params->Initial_E);
        }
        else if (global_index < (initial_points + total_cycle_points)) {
            // 循环段：Scan_Limit1 → Scan_Limit2 → Scan_Limit1，进行多次循环
            uint32_t cycle_global_index = global_index - initial_points;
            uint32_t current_cycle = cycle_global_index / cycle_points;      // 当前是第几个循环
            uint32_t cycle_point_idx = cycle_global_index % cycle_points;    // 当前循环内的点索引
            uint32_t half_cycle_points = cycle_points / 2;

            if (cycle_point_idx < half_cycle_points) {
                // 正向扫描：Scan_Limit1 → Scan_Limit2
                double progress = (half_cycle_points > 1) ? (double)cycle_point_idx / (half_cycle_points - 1) : 0.0;
                voltage = cv_params->Scan_Limit1 + progress * (cv_params->Scan_Limit2 - cv_params->Scan_Limit1);
            } else {
                // 反向扫描：Scan_Limit2 → Scan_Limit1
                uint32_t reverse_index = cycle_point_idx - half_cycle_points;
                uint32_t remaining_points = cycle_points - half_cycle_points;
                double progress = (remaining_points > 1) ? (double)reverse_index / (remaining_points - 1) : 0.0;
                voltage = cv_params->Scan_Limit2 + progress * (cv_params->Scan_Limit1 - cv_params->Scan_Limit2);
            }
        }
        else {
            // 结束阶段：Scan_Limit1 → Final_E
            uint32_t final_index = global_index - initial_points - total_cycle_points;  
            double progress = (final_points > 1) ? (double)final_index / (final_points - 1) : 0.0;
            voltage = cv_params->Scan_Limit1 + progress * (cv_params->Final_E - cv_params->Scan_Limit1);
        }

        // 使用宏进行电位到DAC值的转换
        uint16_t dac_value = ECHEM_VOLTAGE_TO_DAC(voltage);

        // 编码并存储到缓冲区
        encoded_data = control_mask | ((uint32_t)dac_value << 4);
        wave_high_data[i] = (uint16_t)(encoded_data >> 16);
        wave_low_data[i] = (uint16_t)(encoded_data & 0xFFFF);
    }
}

/**
 * @brief  填充下一个CV缓冲区的数据（在主循环中调用）
 * @retval None
 */
void CV_Fill_Next_Buffer(void)
{
    if (!g_pingpong_dma.need_fill_buffer || !g_pingpong_dma.is_running || g_pingpong_dma.transfer_complete) {
        return;
    }
    
    // 计算下次需要填充的起始点
    uint32_t next_start_point = g_pingpong_dma.points_sent + g_pingpong_dma.buffer_size;
    
    if (next_start_point >= g_pingpong_dma.total_points) {
        g_pingpong_dma.need_fill_buffer = false;
        return;
    }
    
    uint32_t remaining_points = g_pingpong_dma.total_points - next_start_point;
    uint32_t points_to_fill = (remaining_points > g_pingpong_dma.buffer_size) ? 
                              g_pingpong_dma.buffer_size : remaining_points;
    
    // 根据标志位指示的缓冲区索引选择目标缓冲区
    if (g_pingpong_dma.buffer_to_fill == 0) {
        Generate_CV_Data_Partial(g_dac_module, g_cv_wave_high_data1, g_cv_wave_low_data1,
                                 next_start_point, points_to_fill, &g_cv_params);
    } else {
        Generate_CV_Data_Partial(g_dac_module, g_cv_wave_high_data2, g_cv_wave_low_data2,
                                 next_start_point, points_to_fill, &g_cv_params);
    }
    
    // 清除填充标志
    g_pingpong_dma.need_fill_buffer = false;
}




/**
 * @brief  循环伏安法DDS输出 - 乒乓DMA精确版本
 * @param  module: DAC80004模块结构体指针
 * @param  cv_params: CV参数结构体指针
 * @param  config: 乒乓DMA配置结构体指针
 * @param  repeat_count: 重复次数 (0=无限循环)
 * @param  wave_high_data1: 乒乓缓冲区1高16位数据
 * @param  wave_high_data2: 乒乓缓冲区2高16位数据
 * @param  wave_low_data1: 乒乓缓冲区1低16位数据
 * @param  wave_low_data2: 乒乓缓冲区2低16位数据
 * @retval true: 成功启动, false: 启动失败
 */
bool CV_DDS_Start_Precise(DAC80004_InitStruct *module,
                         const EchemCV_Params_t *cv_params,
                         const PingPongConfig_t *config,
                         uint16_t *wave_high_data1, uint16_t *wave_high_data2,
                         uint16_t *wave_low_data1, uint16_t *wave_low_data2)
{
    // 参数验证
    if (module == NULL || cv_params == NULL || config == NULL ||
        wave_high_data1 == NULL || wave_high_data2 == NULL || 
        wave_low_data1 == NULL || wave_low_data2 == NULL) {
        
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_INVALID_PARAMS);
        }
        return false;
    }
    
    // 验证CV参数
    if (!ECHEM_IS_VOLTAGE_VALID(cv_params->Initial_E) ||
        !ECHEM_IS_VOLTAGE_VALID(cv_params->Final_E) ||
        !ECHEM_IS_VOLTAGE_VALID(cv_params->Scan_Limit1) ||
        !ECHEM_IS_VOLTAGE_VALID(cv_params->Scan_Limit2) ||
        !ECHEM_IS_SCAN_RATE_VALID(cv_params->Scan_Rate)) {
        
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_INVALID_PARAMS);
        }
        return false;
    }
    
    // 创建可修改的CV参数副本
    EchemCV_Params_t cv_params_calc = *cv_params;
    
    // 保存DAC模块和缓冲区指针到全局变量
    g_dac_module = module;
    g_cv_wave_high_data1 = wave_high_data1;
    g_cv_wave_high_data2 = wave_high_data2;
    g_cv_wave_low_data1 = wave_low_data1;
    g_cv_wave_low_data2 = wave_low_data2;
    
    /* 1. 使用最大采样率计算波形参数 */
    double best_sample_rate = config->max_sample_rate;
    uint32_t cycles = (cv_params->cycles > 0) ? cv_params->cycles : 1;  // 确保至少1个循环
    
    // 计算各段的电位变化和时间
    double initial_voltage_change = fabs(cv_params->Scan_Limit1 - cv_params->Initial_E);
    double theoretical_initial_time = initial_voltage_change / cv_params->Scan_Rate;
    
    double cycle_voltage_change = fabs(cv_params->Scan_Limit2 - cv_params->Scan_Limit1);
    double theoretical_cycle_time = (2.0 * cycle_voltage_change) / cv_params->Scan_Rate;
    
    double final_voltage_change = fabs(cv_params->Final_E - cv_params->Scan_Limit1);
    double theoretical_final_time = final_voltage_change / cv_params->Scan_Rate;
    
    //总时间 = 起始时间 + (单循环时间 × 循环次数) + 结束时间
    double theoretical_total_time = theoretical_initial_time + 
                                   (theoretical_cycle_time * cycles) + 
                                   theoretical_final_time;
    
    // 计算总点数
    uint32_t total_points = (uint32_t)round(best_sample_rate * theoretical_total_time);
    
    // 限制总点数在合理范围内
    if (total_points < config->min_points) {
        total_points = config->min_points;
        best_sample_rate = total_points / theoretical_total_time;
    }
    
    // 重新计算时间比例
    double actual_total_time = total_points / best_sample_rate;
    double time_scale = actual_total_time / theoretical_total_time;
    
    // 计算各段点数并保存到结构体中
    cv_params_calc.initial_points = (uint32_t)round(best_sample_rate * theoretical_initial_time * time_scale);
    cv_params_calc.cycle_points = (uint32_t)round(best_sample_rate * theoretical_cycle_time * time_scale);
    cv_params_calc.final_points = (uint32_t)round(best_sample_rate * theoretical_final_time * time_scale);
    
    // 总点数 = 起始点数 + (单循环点数 × 循环次数) + 结束点数
    uint32_t calculated_total = cv_params_calc.initial_points + 
                               (cv_params_calc.cycle_points * cycles) + 
                               cv_params_calc.final_points;
    
    // 调整点数确保总和正确
    if (calculated_total != total_points) {
        int32_t diff = total_points - calculated_total;
        cv_params_calc.cycle_points += (diff / cycles);  // 平均分配到各个循环
        
        // 重新计算总点数
        calculated_total = cv_params_calc.initial_points + 
                          (cv_params_calc.cycle_points * cycles) + 
                          cv_params_calc.final_points;
    }
    
    // 确保各段点数不为0
    if (cv_params_calc.initial_points == 0 && initial_voltage_change > 0) 
        cv_params_calc.initial_points = 1;
    if (cv_params_calc.cycle_points < 2 && cycle_voltage_change > 0) 
        cv_params_calc.cycle_points = 2;
    if (cv_params_calc.final_points == 0 && final_voltage_change > 0) 
        cv_params_calc.final_points = 1;
    
    // 重新计算总点数并保存到结构体
    cv_params_calc.total_points = cv_params_calc.initial_points + 
                                 (cv_params_calc.cycle_points * cycles) + 
                                 cv_params_calc.final_points;
    cv_params_calc.actual_sample_rate = best_sample_rate;
    
    // 保存计算后的CV参数到全局变量
    g_cv_params = cv_params_calc;
    
    /* 2. 初始化乒乓DMA管理结构体 */
    g_pingpong_dma.active_buffer = 0;
    g_pingpong_dma.total_points = cv_params_calc.total_points;
    g_pingpong_dma.points_sent = 0;
    g_pingpong_dma.buffer_size = config->buffer_size;
    g_pingpong_dma.transfer_complete = false;
    g_pingpong_dma.is_running = true;
    g_pingpong_dma.need_fill_buffer = false;
    g_pingpong_dma.buffer_to_fill = 0;
    
    /* 3. 更新实验结果结构体 */
    g_echem_result.total_points = cv_params_calc.total_points;
    g_echem_result.actual_sample_rate = best_sample_rate;
    g_echem_result.total_duration = cv_params_calc.total_points / best_sample_rate;
    g_echem_result.actual_scan_rate = cycle_voltage_change / (cv_params_calc.cycle_points / best_sample_rate / 2.0);
    g_echem_result.scan_rate_error = fabs(g_echem_result.actual_scan_rate - cv_params->Scan_Rate);
    g_echem_result.error_percent = (g_echem_result.scan_rate_error / cv_params->Scan_Rate) * 100.0;
    g_echem_result.method = ECHEM_METHOD_CV;
    g_echem_result.state = ECHEM_STATE_PREPARING;
    g_echem_result.success = false;
    g_echem_result.timestamp = LL_SYSTICK_GetClkSource();
    
    /* 4. 填充第一个缓冲区（缓冲区1） */
    uint32_t first_batch_size = (cv_params_calc.total_points > config->buffer_size) ? config->buffer_size : cv_params_calc.total_points;
    Generate_CV_Data_Partial(module, wave_high_data1, wave_low_data1,
                             0, first_batch_size, &cv_params_calc);
    
    /* 5. 如果数据量大于一个缓冲区，填充第二个缓冲区（缓冲区2） */
    if (cv_params_calc.total_points > config->buffer_size) {
        uint32_t second_batch_size = ((cv_params_calc.total_points - config->buffer_size) > config->buffer_size) ? 
                                     config->buffer_size : (cv_params_calc.total_points - config->buffer_size);
        Generate_CV_Data_Partial(module, wave_high_data2, wave_low_data2,
                                 config->buffer_size, second_batch_size, &cv_params_calc);
    }
    
    /* 6. 配置DMA使用乒乓缓冲区模式 */
    // 等待SPI空闲
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
    while(LL_SPI_IsActiveFlag_BSY(SPI1)) {}
    
    // 停止当前DMA传输
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4)) {}
    
    // 清除DMA标志位
    LL_DMA_ClearFlag_TE5(DMA2);
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_DME5(DMA2);
    LL_DMA_ClearFlag_FE5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);
    
    LL_DMA_ClearFlag_TE4(DMA2);
    LL_DMA_ClearFlag_TC4(DMA2);
    LL_DMA_ClearFlag_DME4(DMA2);
    LL_DMA_ClearFlag_FE4(DMA2);
    LL_DMA_ClearFlag_HT4(DMA2);
    
    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_TIM_ClearFlag_CC4(TIM1);
    
    
    // 配置DMA为正常模式（非循环）
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_high_data1);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, first_batch_size);
    
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)wave_low_data1);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, first_batch_size);
    
    // 使能DMA传输完成中断
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_4);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    
    /* 7. 配置定时器频率 */
    TIM_FreqConfig_t freq_config;
    uint32_t timer_clock = 100000000;
    
    if (TIM_CalculateFreqDivision_Precise(timer_clock, best_sample_rate, &freq_config) != 0) {
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_TIMER_CONFIG);
        }
        return false;
    }
    
    TIM_ApplyFreqConfig_DualDMA(TIM1, &freq_config, 100000000, 100000000/32);
    SYNC_Cycle_SetPara(&freq_config, 100000000, 100000000/32);
    
    /* 8. 启动传输 */
    g_echem_result.state = ECHEM_STATE_RUNNING;
    
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_TIM_EnableDMAReq_UPDATE(TIM1);
    LL_TIM_EnableDMAReq_CC4(TIM1);
    
    // 启动DMA流
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
    
    // 启动定时器和同步信号
    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_TIM_ClearFlag_CC4(TIM1);
    
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    
    SYNC_Cycle_Start();
    LL_TIM_EnableCounter(TIM1);
    
    g_echem_result.success = true;
    
    return true;
}




/**
 * @brief  检查是否需要填充缓冲区
 * @retval true: 需要填充, false: 不需要填充
 */
bool CV_NeedFillBuffer(void)
{
    return g_pingpong_dma.need_fill_buffer && g_pingpong_dma.is_running && !g_pingpong_dma.transfer_complete;
}

/**
 * @brief  乒乓DMA中断处理函数 - Stream5（高16位）
 */
void CV_PingPong_DMA2_Stream5_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC5(DMA2)) {
        LL_DMA_ClearFlag_TC5(DMA2);
        // Stream5的传输完成处理在Stream4中完成（Stream4通常后完成）
    }
    
    if (LL_DMA_IsActiveFlag_TE5(DMA2)) {
        LL_DMA_ClearFlag_TE5(DMA2);
        g_pingpong_dma.is_running = false;
        g_pingpong_dma.need_fill_buffer = false;
        g_echem_result.state = ECHEM_STATE_ERROR;
        
        PingPong_DMA_Stop();
        
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_DMA_FAILURE);
        }
    }
}

/**
 * @brief  乒乓DMA中断处理函数 - Stream4（低16位）- 主处理逻辑
 */
void CV_PingPong_DMA2_Stream4_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
        LL_DMA_ClearFlag_TC4(DMA2);
        
        if (!g_pingpong_dma.is_running) {
            return; // 如果已停止运行，直接返回
        }
        
        // 更新已发送点数
        g_pingpong_dma.points_sent += g_pingpong_dma.buffer_size;
        
        // 调用进度回调函数
        if (g_progress_callback != NULL) {
            uint8_t progress = PingPong_DMA_GetProgress();
            g_progress_callback(progress, ECHEM_STATE_RUNNING);
        }
        
        // 检查是否传输完成
        if (g_pingpong_dma.points_sent >= g_pingpong_dma.total_points) {
            // 传输完成，停止DMA
            g_pingpong_dma.transfer_complete = true;
            g_pingpong_dma.is_running = false;
            g_echem_result.state = ECHEM_STATE_COMPLETED;
            
            PingPong_DMA_Stop();
            
            if (g_progress_callback != NULL) {
                g_progress_callback(100, ECHEM_STATE_COMPLETED);
            }
            return;
        }
        
        // 切换到另一个缓冲区
        g_pingpong_dma.active_buffer = 1 - g_pingpong_dma.active_buffer;
        
        // ======== 关键：设置填充标志位，让主循环填充下一个缓冲区 ========
        uint32_t next_start_point = g_pingpong_dma.points_sent + g_pingpong_dma.buffer_size;
        if (next_start_point < g_pingpong_dma.total_points) {
            g_pingpong_dma.buffer_to_fill = g_pingpong_dma.active_buffer;  // 设置要填充的缓冲区
            g_pingpong_dma.need_fill_buffer = true;  // 设置填充标志
        }
        
        // 重新配置DMA指向新的活跃缓冲区
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
        
        while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5) || LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4)) {}
        
        // 计算剩余点数
        uint32_t remaining_points = g_pingpong_dma.total_points - g_pingpong_dma.points_sent;
        uint32_t next_transfer_size = (remaining_points > g_pingpong_dma.buffer_size) ? 
                                      g_pingpong_dma.buffer_size : remaining_points;
        
        // 设置新的DMA地址和长度
        if (g_pingpong_dma.active_buffer == 0) {
            LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)g_cv_wave_high_data1);
            LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)g_cv_wave_low_data1);
        } else {
            LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)g_cv_wave_high_data2);
            LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)g_cv_wave_low_data2);
        }
        
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, next_transfer_size);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, next_transfer_size);
        
        // 重新启动DMA
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
    }
    
    if (LL_DMA_IsActiveFlag_TE4(DMA2)) {
        LL_DMA_ClearFlag_TE4(DMA2);
        g_pingpong_dma.is_running = false;
        g_pingpong_dma.need_fill_buffer = false;
        g_echem_result.state = ECHEM_STATE_ERROR;
        
        PingPong_DMA_Stop();
        
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_DMA_FAILURE);
        }
    }
}



// ==================== 实验结果和回调管理 ====================

void Echem_stim_Init(DAC80004_InitStruct *module)
{
    DAC80004_Init(module);// 初始化GPIO
    MX_DMA_Init();
    TIM1_DMA_SPI1_Init();
    SYNC_Cycle_Init(); // 初始化SYNC周期信号
}

/**
 * @brief  重置实验状态
 * @retval None
 */
void Echem_Reset(void)
{
    PingPong_DMA_Reset();
    
    // 重置CV参数结构体
    memset(&g_cv_params, 0, sizeof(EchemCV_Params_t));
    
    memset(&g_echem_result, 0, sizeof(EchemResult_t));
    g_echem_result.state = ECHEM_STATE_IDLE;
}


/**
 * @brief  获取实验结果
 * @retval 实验结果结构体指针
 */
const EchemResult_t* Echem_GetResult(void)
{
    return &g_echem_result;
}

/**
 * @brief  设置进度回调函数
 * @param  callback: 进度回调函数指针
 * @retval None
 */
void Echem_SetProgressCallback(EchemProgressCallback_t callback)
{
    g_progress_callback = callback;
}

/**
 * @brief  设置错误回调函数
 * @param  callback: 错误回调函数指针
 * @retval None
 */
void Echem_SetErrorCallback(EchemErrorCallback_t callback)
{
    g_error_callback = callback;
}



/**
 * @brief  DMA2 Stream5中断处理函数 - 支持乒乓DMA
 */
void DMA2_Stream5_IRQHandler(void)
{
    if (g_pingpong_dma.is_running) {
        CV_PingPong_DMA2_Stream5_IRQHandler();
    }
    // switch (g_dma_handler.current_mode)
    // {
    // case WAVE_MODE_SINE:
    //     Sin_Wave_DMA2_Stream5_IRQHandler();
    //     break;
    // case WAVE_MODE_CV:
    //     // 检查是否使用乒乓DMA模式
    //     if (g_pingpong_dma.is_running) {
    //         CV_PingPong_DMA2_Stream5_IRQHandler();
    //     } else {
    //         CV_Wave_DMA2_Stream5_IRQHandler();
    //     }
    //     break;
    // default:
    //     break;
    // }
}

/**
 * @brief  DMA2 Stream4中断处理函数 - 支持乒乓DMA
 */
void DMA2_Stream4_IRQHandler(void)
{
    if (g_pingpong_dma.is_running) {
        CV_PingPong_DMA2_Stream4_IRQHandler();
    }
    // switch (g_dma_handler.current_mode)
    // {
    // case WAVE_MODE_SINE:
    //     Sin_Wave_DMA2_Stream4_IRQHandler();
    //     break;
    // case WAVE_MODE_CV:
    //     // 检查是否使用乒乓DMA模式
    //     if (g_pingpong_dma.is_running) {
    //         CV_PingPong_DMA2_Stream4_IRQHandler();
    //     } else {
    //         CV_Wave_DMA2_Stream4_IRQHandler();
    //     }
    //     break;
    // default:
    //     break;
    // }
}