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

// 回调函数指针
static EchemProgressCallback_t g_progress_callback = NULL;
static EchemErrorCallback_t g_error_callback = NULL;

// 静态缓冲区指针（供中断使用）
static uint16_t *g_cv_wave_high_data1 = NULL;
static uint16_t *g_cv_wave_high_data2 = NULL;
static uint16_t *g_cv_wave_low_data1 = NULL;
static uint16_t *g_cv_wave_low_data2 = NULL;

/**
 * @brief  生成CV波形数据到指定缓冲区
 * @param  module: DAC80004模块结构体指针
 * @param  wave_high_data: 高16位数据缓冲区
 * @param  wave_low_data: 低16位数据缓冲区
 * @param  start_point: 起始点索引
 * @param  points_to_fill: 要填充的点数
 * @param  total_points: 总点数
 * @param  cv_params: CV参数结构体指针
 * @param  initial_points: 起始段点数
 * @param  cycle_points: 循环段点数
 * @param  final_points: 结束段点数
 * @retval None
 */
static void Generate_CV_Data_Partial(DAC80004_InitStruct *module,
                                    uint16_t *wave_high_data, uint16_t *wave_low_data,
                                    uint32_t start_point, uint32_t points_to_fill, uint32_t total_points,
                                    const EchemCV_Params_t *cv_params,
                                    uint32_t initial_points, uint32_t cycle_points, uint32_t final_points)
{
    uint32_t control_mask = module->TX_Data & ~(0xFFFF << 4); // 数据传输控制掩码
    uint32_t encoded_data;
    
    for (uint32_t i = 0; i < points_to_fill; i++) {
        uint32_t global_index = start_point + i; // 全局数据点索引
        double voltage = 0.0;
        
        // 根据全局索引确定当前处于哪个阶段，并计算对应的电压值
        if (global_index < initial_points) {
            // 起始阶段：Initial_E → Scan_Limit1
            double progress = (initial_points > 1) ? (double)global_index / (initial_points - 1) : 0.0;
            voltage = cv_params->Initial_E + progress * (cv_params->Scan_Limit1 - cv_params->Initial_E);
        }
        else if (global_index < (initial_points + cycle_points)) {
            // 循环阶段：Scan_Limit1 → Scan_Limit2 → Scan_Limit1
            uint32_t cycle_index = global_index - initial_points;
            uint32_t half_cycle_points = cycle_points / 2;
            
            if (cycle_index < half_cycle_points) {
                // 正向扫描：Scan_Limit1 → Scan_Limit2
                double progress = (half_cycle_points > 1) ? (double)cycle_index / (half_cycle_points - 1) : 0.0;
                voltage = cv_params->Scan_Limit1 + progress * (cv_params->Scan_Limit2 - cv_params->Scan_Limit1);
            } else {
                // 反向扫描：Scan_Limit2 → Scan_Limit1
                uint32_t reverse_index = cycle_index - half_cycle_points;
                uint32_t remaining_points = cycle_points - half_cycle_points;
                double progress = (remaining_points > 1) ? (double)reverse_index / (remaining_points - 1) : 0.0;
                voltage = cv_params->Scan_Limit2 + progress * (cv_params->Scan_Limit1 - cv_params->Scan_Limit2);
            }
        }
        else {
            // 结束阶段：Scan_Limit1 → Final_E
            uint32_t final_index = global_index - initial_points - cycle_points;
            double progress = (final_points > 1) ? (double)final_index / (final_points - 1) : 0.0;
            voltage = cv_params->Scan_Limit1 + progress * (cv_params->Final_E - cv_params->Scan_Limit1);
        }
        
        // 使用宏进行电位到DAC值的转换
        uint16_t dac_value = ECHEM_VOLTAGE_TO_DAC(voltage);
        
        // 编码并存储到缓冲区
        encoded_data = control_mask | ((uint32_t)dac_value << 4);
        wave_high_data[i] = (uint16_t)(encoded_data >> 16);      // 高16位
        wave_low_data[i] = (uint16_t)(encoded_data & 0xFFFF);    // 低16位
    }
}

/**
 * @brief  乒乓DMA中断处理函数 - Stream5（高16位）
 */
void CV_PingPong_DMA2_Stream5_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC5(DMA2)) {
        LL_DMA_ClearFlag_TC5(DMA2);
        
        if (!g_pingpong_dma.is_running) {
            return; // 如果已停止运行，直接返回
        }
        
        // 更新已发送点数
        g_pingpong_dma.points_sent += g_pingpong_dma.buffer_size;
        
        // 调用进度回调函数
        if (g_progress_callback != NULL) {
            uint8_t progress = CV_PingPong_GetProgress();
            g_progress_callback(progress, ECHEM_STATE_RUNNING);
        }
        
        // 检查是否传输完成
        if (g_pingpong_dma.points_sent >= g_pingpong_dma.total_points) {
            // 传输完成，停止DMA
            g_pingpong_dma.transfer_complete = true;
            g_pingpong_dma.is_running = false;
            g_echem_result.state = ECHEM_STATE_COMPLETED;
            
            CV_PingPong_Stop_Transfer();
            
            // 调用完成回调
            if (g_progress_callback != NULL) {
                g_progress_callback(100, ECHEM_STATE_COMPLETED);
            }
            return;
        }
        
        // 切换到另一个缓冲区
        g_pingpong_dma.active_buffer = 1 - g_pingpong_dma.active_buffer;
        
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
    
    if (LL_DMA_IsActiveFlag_TE5(DMA2)) {
        LL_DMA_ClearFlag_TE5(DMA2);
        g_pingpong_dma.is_running = false;
        g_echem_result.state = ECHEM_STATE_ERROR;
        
        CV_PingPong_Stop_Transfer();
        
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_DMA_FAILURE);
        }
    }
}

/**
 * @brief  乒乓DMA中断处理函数 - Stream4（低16位）
 */
void CV_PingPong_DMA2_Stream4_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
        LL_DMA_ClearFlag_TC4(DMA2);
        // Stream4的传输完成处理已在Stream5中完成
    }
    
    if (LL_DMA_IsActiveFlag_TE4(DMA2)) {
        LL_DMA_ClearFlag_TE4(DMA2);
        g_pingpong_dma.is_running = false;
        g_echem_result.state = ECHEM_STATE_ERROR;
        
        CV_PingPong_Stop_Transfer();
        
        if (g_error_callback != NULL) {
            g_error_callback(ECHEM_STATE_ERROR, ECHEM_ERROR_DMA_FAILURE);
        }
    }
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
                         uint32_t repeat_count,
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
    
    // 保存缓冲区指针到全局变量（供中断使用）
    g_cv_wave_high_data1 = wave_high_data1;
    g_cv_wave_high_data2 = wave_high_data2;
    g_cv_wave_low_data1 = wave_low_data1;
    g_cv_wave_low_data2 = wave_low_data2;
    
    /* 1. 使用最大采样率计算波形参数 */
    double best_sample_rate = config->max_sample_rate;
    
    // 计算各段的电位变化和时间
    double initial_voltage_change = fabs(cv_params->Scan_Limit1 - cv_params->Initial_E);
    double theoretical_initial_time = initial_voltage_change / cv_params->Scan_Rate;
    
    double cycle_voltage_change = fabs(cv_params->Scan_Limit2 - cv_params->Scan_Limit1);
    double theoretical_cycle_time = (2.0 * cycle_voltage_change) / cv_params->Scan_Rate;
    
    double final_voltage_change = fabs(cv_params->Final_E - cv_params->Scan_Limit1);
    double theoretical_final_time = final_voltage_change / cv_params->Scan_Rate;
    
    double theoretical_total_time = theoretical_initial_time + theoretical_cycle_time + theoretical_final_time;
    
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
    
    // 计算各段点数
    uint32_t initial_points = (uint32_t)round(best_sample_rate * theoretical_initial_time * time_scale);
    uint32_t cycle_points = (uint32_t)round(best_sample_rate * theoretical_cycle_time * time_scale);
    uint32_t final_points = (uint32_t)round(best_sample_rate * theoretical_final_time * time_scale);
    
    // 调整点数确保总和正确
    uint32_t calculated_total = initial_points + cycle_points + final_points;
    if (calculated_total != total_points) {
        int32_t diff = total_points - calculated_total;
        cycle_points += diff; // 优先调整循环部分
    }
    
    // 确保各段点数不为0
    if (initial_points == 0 && initial_voltage_change > 0) initial_points = 1;
    if (cycle_points < 2 && cycle_voltage_change > 0) cycle_points = 2;
    if (final_points == 0 && final_voltage_change > 0) final_points = 1;
    
    // 重新计算总点数
    total_points = initial_points + cycle_points + final_points;
    
    /* 2. 初始化乒乓DMA管理结构体 */
    g_pingpong_dma.active_buffer = 0;
    g_pingpong_dma.total_points = total_points;
    g_pingpong_dma.points_sent = 0;
    g_pingpong_dma.buffer_size = config->buffer_size;
    g_pingpong_dma.transfer_complete = false;
    g_pingpong_dma.is_running = true;
    
    /* 3. 更新实验结果结构体 */
    g_echem_result.total_points = total_points;
    g_echem_result.actual_sample_rate = best_sample_rate;
    g_echem_result.total_duration = total_points / best_sample_rate;
    g_echem_result.actual_scan_rate = cycle_voltage_change / (cycle_points / best_sample_rate / 2.0);
    g_echem_result.scan_rate_error = fabs(g_echem_result.actual_scan_rate - cv_params->Scan_Rate);
    g_echem_result.error_percent = (g_echem_result.scan_rate_error / cv_params->Scan_Rate) * 100.0;
    g_echem_result.method = ECHEM_METHOD_CV;
    g_echem_result.state = ECHEM_STATE_PREPARING;
    g_echem_result.success = false;
    g_echem_result.timestamp = LL_SYSTICK_GetClkSource(); // 简单的时间戳
    
    /* 4. 填充第一个缓冲区（缓冲区1） */
    uint32_t first_batch_size = (total_points > config->buffer_size) ? config->buffer_size : total_points;
    Generate_CV_Data_Partial(module, wave_high_data1, wave_low_data1,
                             0, first_batch_size, total_points,
                             cv_params, initial_points, cycle_points, final_points);
    
    /* 5. 如果数据量大于一个缓冲区，填充第二个缓冲区（缓冲区2） */
    if (total_points > config->buffer_size) {
        uint32_t second_batch_size = ((total_points - config->buffer_size) > config->buffer_size) ? 
                                     config->buffer_size : (total_points - config->buffer_size);
        Generate_CV_Data_Partial(module, wave_high_data2, wave_low_data2,
                                 config->buffer_size, second_batch_size, total_points,
                                 cv_params, initial_points, cycle_points, final_points);
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
    
    // 初始化DMA处理器
    DMA_Handler_Init();
    DMA_Handler_SetData(WAVE_MODE_CV, wave_high_data1, first_batch_size, 
                       wave_low_data1, first_batch_size, repeat_count);
    
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
    
    TIM_ApplyFreqConfig_DualDMA(TIM1, &freq_config, 100000000, 100000000/2);
    SYNC_Cycle_SetPara(&freq_config, 100000000, 100000000/2);
    
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
 * @brief  填充下一个乒乓缓冲区的数据（在主循环中调用）
 * @param  module: DAC80004模块结构体指针
 * @param  cv_params: CV参数结构体指针
 * @param  initial_points: 起始段点数
 * @param  cycle_points: 循环段点数
 * @param  final_points: 结束段点数
 * @retval None
 */
void CV_Fill_Next_Buffer(DAC80004_InitStruct *module,
                        const EchemCV_Params_t *cv_params,
                        uint32_t initial_points, uint32_t cycle_points, uint32_t final_points)
{
    if (!g_pingpong_dma.is_running || g_pingpong_dma.transfer_complete) {
        return; // 传输未运行或已完成
    }
    
    // 计算下次需要填充的缓冲区
    uint8_t next_buffer = 1 - g_pingpong_dma.active_buffer;
    uint32_t next_start_point = g_pingpong_dma.points_sent + g_pingpong_dma.buffer_size;
    
    if (next_start_point >= g_pingpong_dma.total_points) {
        return; // 没有更多数据需要填充
    }
    
    uint32_t remaining_points = g_pingpong_dma.total_points - next_start_point;
    uint32_t points_to_fill = (remaining_points > g_pingpong_dma.buffer_size) ? 
                              g_pingpong_dma.buffer_size : remaining_points;
    
    // 根据缓冲区索引选择目标缓冲区
    if (next_buffer == 0) {
        Generate_CV_Data_Partial(module, g_cv_wave_high_data1, g_cv_wave_low_data1,
                                 next_start_point, points_to_fill, g_pingpong_dma.total_points,
                                 cv_params, initial_points, cycle_points, final_points);
    } else {
        Generate_CV_Data_Partial(module, g_cv_wave_high_data2, g_cv_wave_low_data2,
                                 next_start_point, points_to_fill, g_pingpong_dma.total_points,
                                 cv_params, initial_points, cycle_points, final_points);
    }
}

/**
 * @brief  检查乒乓DMA传输状态
 * @retval true: 传输完成, false: 传输进行中
 */
bool CV_PingPong_IsComplete(void)
{
    return g_pingpong_dma.transfer_complete;
}

/**
 * @brief  获取乒乓DMA传输进度
 * @retval 传输进度百分比 (0-100)
 */
uint8_t CV_PingPong_GetProgress(void)
{
    if (g_pingpong_dma.total_points == 0) {
        return 0;
    }
    
    return (uint8_t)((g_pingpong_dma.points_sent * 100) / g_pingpong_dma.total_points);
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
 * @brief  重置实验状态
 * @retval None
 */
void Echem_Reset(void)
{
    g_pingpong_dma.active_buffer = 0;
    g_pingpong_dma.total_points = 0;
    g_pingpong_dma.points_sent = 0;
    g_pingpong_dma.buffer_size = 0;
    g_pingpong_dma.transfer_complete = false;
    g_pingpong_dma.is_running = false;
    
    memset(&g_echem_result, 0, sizeof(EchemResult_t));
    g_echem_result.state = ECHEM_STATE_IDLE;
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