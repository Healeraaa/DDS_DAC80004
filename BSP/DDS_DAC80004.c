#include "DDS_DAC80004.h"
#include "spi.h"
// #include "tim.h"
#include "dma.h"
#include <math.h>       

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile DMAHandler_t g_dma_handler = {0};
SineWaveResult_t SineWaveResult = {0};
CvWaveResult_t CvWaveResult = {0};

void DMA_Handler_Init(void)
{
    // 清零整个结构体
    memset((void*)&g_dma_handler, 0, sizeof(DMAHandler_t));
    
    // 设置初始状态
    g_dma_handler.current_mode = WAVE_MODE_NONE;
    g_dma_handler.state = DMA_STATE_IDLE;
}

/**
 * @brief  配置DMA数据
 */
void DMA_Handler_SetData(WaveMode_t mode,uint16_t *high_data, uint32_t high_points,
                        uint16_t *low_data, uint32_t low_points,
                        uint32_t repeat_count)
{
    g_dma_handler.current_mode = mode;
    // 配置Stream5数据
    g_dma_handler.stream5.data_ptr = high_data;
    g_dma_handler.stream5.data_points = high_points;
    g_dma_handler.stream5.current_count = 0;
    g_dma_handler.stream5.transfer_complete = 0;
    
    // 配置Stream4数据
    g_dma_handler.stream4.data_ptr = low_data;
    g_dma_handler.stream4.data_points = low_points;
    g_dma_handler.stream4.current_count = 0;
    g_dma_handler.stream4.transfer_complete = 0;
    
    // 配置模式参数
    g_dma_handler.repeat_count = repeat_count; // DMA循环次数；0=无限循环
    g_dma_handler.wave_stage = WAVE_STAGE_STAGE1; // 初始阶段1

    
    g_dma_handler.state = DMA_STATE_RUNNING;
}

void SYNC_Cycle_Init(void)                                                      // 初始化SYNC周期信号，内部调用TIM3 PWM初始化
{
    TIM3_PWM_Init();                                                           // 调用TIM3 PWM初始化函数，配置硬件但不启动
}
void SYNC_Cycle_SetPara(const TIM_FreqConfig_t *config, double timer_clock, double spi_baudrate)  // 设置SYNC信号参数
{
    TIM3_ApplyPWMConfig(config, timer_clock, spi_baudrate);                    // 应用频率配置，设置CS信号低电平持续35个SPI时钟周期
}
void SYNC_Cycle_Start(void)                                                    // 启动SYNC周期信号输出
{
    TIM3_PWM_Start();                                                          // 启动TIM3计数器，开始产生CS信号
}
void SYNC_Cycle_Stop(void)                                                     // 停止SYNC周期信号输出
{
    TIM3_PWM_Stop();                                                           // 停止TIM3计数器，清零计数值，CS保持高电平
}
void DDS_Init(DAC80004_InitStruct *module)
{
    DAC80004_Init(module);// 初始化GPIO
    MX_DMA_Init();
    TIM1_DMA_SPI1_Init();
    SYNC_Cycle_Init(); // 初始化SYNC周期信号
}

/**
 * @brief  DDS启动（双DMA模式）- 一个定时器周期传输4个数据
 * @param  wave_data: 波形数据缓冲区（交替存储两个DMA流的数据）
 * @param  data_pairs: 数据对数（每对包含4个16位数据）
 * @param  sample_rate: 采样率 (Hz)
 * @retval true: 成功, false: 失败
 */
bool DDS_Start_DualDMA(DAC80004_InitStruct *module,WaveMode_t wavemode ,uint16_t *wave_data_high,
                        uint16_t *wave_data_low, uint16_t data_high_size,uint16_t data_low_size,
                        double sample_rate, uint32_t repeat_count)
{
    if (wave_data_high == NULL || wave_data_low == NULL || data_high_size == 0 || data_low_size == 0) {
        return false;
    }
        // 等待SPI空闲
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
    while(LL_SPI_IsActiveFlag_BSY(SPI1)) {}
    
    /* 1. 停止当前DMA传输 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4)) {}
    
    /* 2. 清除DMA标志位 */
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

    LL_TIM_ClearFlag_UPDATE(TIM1);                          // 清除UPDATE事件标志位
    LL_TIM_ClearFlag_CC4(TIM1);                             // 清除CC4比较事件标志位
    
    
    
    // /* 3. 设置循环参数 - 修改为分离管理 */

    DMA_Handler_Init(); // 初始化DMA处理器
    DMA_Handler_SetData(wavemode, wave_data_high, data_high_size, wave_data_low, data_low_size, repeat_count);
    
    /* 4. 配置DMA */
    if (repeat_count == 0) {
        // 无限循环模式
        // 配置Stream5（UPDATE事件触发）- 传输高16位数据
        LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_data_high);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, data_high_size);
        
        // 配置Stream4（CC4事件触发）- 传输低16位数据
        LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)wave_data_low);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, data_low_size);
    } 
    else {
        // 有限次数模式 - 使用普通模式，每次传输完成后重新启动
        // 配置Stream5（UPDATE事件触发）- 传输高16位数据
        LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_data_high);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, data_high_size);
        
        // 配置Stream4（CC1事件触发）- 传输低16位数据
        LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)wave_data_low);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, data_low_size);
        
        // 使能DMA传输完成中断（只需要监控一个流完成即可）
        LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
        LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_5);
        LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
        LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_4);  // 也监控Stream4的错误
        NVIC_EnableIRQ(DMA2_Stream5_IRQn);
        NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    }
    
    /* 5. 配置定时器频率 */
    TIM_FreqConfig_t freq_config;
    uint32_t timer_clock = 100000000;
    
    if (TIM_CalculateFreqDivision_Precise(timer_clock, sample_rate, &freq_config) != 0) {
        return false;
    }
    
    // TIM_ApplyFreqConfig(TIM1, &freq_config);
    // TIM_ApplyFreqConfig_DualDMA(TIM1, &freq_config);
    TIM_ApplyFreqConfig_DualDMA(TIM1, &freq_config,100000000,100000000/4);
    SYNC_Cycle_SetPara(&freq_config,100000000,100000000/4); // 设置SYNC信号参数
    
    /* 6. 启动传输 */
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_TIM_EnableDMAReq_UPDATE(TIM1);  // 使能UPDATE事件DMA请求
    LL_TIM_EnableDMAReq_CC4(TIM1);     // 使能CC4事件DMA请求
    // 启动两个DMA流

    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);  // Stream5由UPDATE触发
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);  // Stream4由CC1触发
    // 启动定时器
    
  
    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_TIM_ClearFlag_CC4(TIM1);
    
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM1);  // 触发一次UPDATE事件，确保第一个数据及时传输
    

    SYNC_Cycle_Start();               // 启动SYNC信号
    LL_TIM_EnableCounter(TIM1);
    
    return true;
}

/**
 * @brief  停止双DMA DDS输出
 */
void DDS_Stop_DualDMA(void)
{
    LL_TIM_DisableCounter(TIM1);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_TIM_DisableDMAReq_UPDATE(TIM1);
    LL_TIM_DisableDMAReq_CC4(TIM1);
    LL_TIM_SetCounter(TIM1, 0);

    // 清除完成标志
    g_dma_handler.stream5.transfer_complete = 0;        // 清除高16位完成标志                                                                                                                                                                        
    g_dma_handler.stream4.transfer_complete = 0;        // 清除低16位完成标志
    
    // 禁用中断
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_4);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_5);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_4);
}

/**
 * @brief  智能正弦波生成 - 优化版，减少计算量
 * @param  wave_buffer: 输出缓冲区
 * @param  target_freq: 目标频率 (Hz)
 * @param  max_sample_rate: 系统最大允许采样率
 * @param  min_points: 最小点数要求
 * @param  max_points: 最大点数限制
 * @param  amplitude: 幅值 (0-32767)
 * @param  offset: 直流偏移 (0-65535)
 * @param  result: 输出结果结构体指针
 * @retval None
 */
void Generate_Smart_Sine_Wave(uint16_t *wave_buffer,
                              double target_freq, double max_sample_rate,
                              uint32_t min_points, uint32_t max_points,
                              uint16_t amplitude, uint16_t offset,
                              SineWaveResult_t *result)
{
    // 初始化结果结构体
    if (result != NULL) {
        result->points = 0;
        result->actual_freq = 0.0;
        result->actual_sample_rate = 0.0;
        result->frequency_error = 0.0;
        result->error_percent = 0.0;
        result->success = false;
    }
    
    // 参数验证
    if (wave_buffer == NULL || result == NULL || target_freq <= 0 || max_sample_rate <= 0 || 
        min_points == 0 || max_points < min_points) {
        return;  // result->success 已经是 false
    }
    
    /* 1. 计算理论最大采样率 - 基于目标频率和最大点数 */
    double theoretical_max_sample_rate = target_freq * max_points;  // 理论最大采样率
    
    /* 2. 选择实际使用的最大采样率 - 取较小值，避免过度计算 */
    double effective_max_sample_rate = (theoretical_max_sample_rate < max_sample_rate) ? 
                                       theoretical_max_sample_rate : max_sample_rate;
    
    /* 3. 计算搜索的下限采样率 */
    double min_sample_rate = target_freq * min_points;  // 最小采样率
    
    /* 4. 计算搜索步长 - 基于目标频率，避免过小的步长 */
    double search_step = target_freq * 0.1;  // 步长为目标频率的10%，减少计算量
    if (search_step < 1.0) search_step = 1.0;  // 最小步长为1Hz
    
    double best_sample_rate = effective_max_sample_rate;
    uint32_t best_points = 0;
    double min_error = 1e9;
    
    /* 5. 优化的搜索循环 - 从有效最大采样率向下搜索 */
    for (double sr = effective_max_sample_rate; sr >= min_sample_rate; sr -= search_step) {
        uint32_t points = (uint32_t)round(sr / target_freq);  // 计算点数
        
        // 检查点数是否在合理范围内
        if (points >= min_points && points <= max_points) {
            double actual_f = sr / points;                    // 实际频率
            double error = fabs(actual_f - target_freq);      // 频率误差
            
            if (error < min_error) {
                min_error = error;
                best_sample_rate = sr;
                best_points = points;
                
                // 如果误差足够小（精度达到0.0001%），提前退出
                if (error < target_freq * 1e-6) break;
            }
        }
    }
    
    /* 6. 如果粗搜索没找到合适的，进行精细搜索 */
    if (best_points == 0) {
        // 在最优采样率附近进行精细搜索
        double center_sr = target_freq * ((min_points + max_points) / 2);  // 中心采样率
        double fine_search_range = target_freq * 100;  // 精细搜索范围
        double fine_step = target_freq * 0.01;          // 精细搜索步长
        
        for (double sr = center_sr - fine_search_range; 
             sr <= center_sr + fine_search_range && sr <= max_sample_rate; 
             sr += fine_step) {
            
            if (sr < min_sample_rate) continue;  // 跳过过小的采样率
            
            uint32_t points = (uint32_t)round(sr / target_freq);
            
            if (points >= min_points && points <= max_points) {
                double actual_f = sr / points;
                double error = fabs(actual_f - target_freq);
                
                if (error < min_error) {
                    min_error = error;
                    best_sample_rate = sr;
                    best_points = points;
                }
            }
        }
    }
    
    /* 7. 如果还是没找到合适的，使用备用方案 */
    if (best_points == 0) {
        best_points = (uint32_t)round(effective_max_sample_rate / target_freq);
        if (best_points < min_points) best_points = min_points;
        if (best_points > max_points) best_points = max_points;
        best_sample_rate = target_freq * best_points;
    }
    
    /* 8. 生成正弦波数据 */
    for (uint32_t i = 0; i < best_points; i++) {
        double phase = (2.0 * M_PI * i) / best_points;        // 计算相位
        double sine_value = sin(phase);                       // 计算正弦值
        
        // 转换为DAC值：偏移 + 幅值 * 正弦值
        int32_t dac_value = offset + (int32_t)(amplitude * sine_value);
        
        // 限幅处理
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        
        wave_buffer[i] = (uint16_t)dac_value;
    }
    
    /* 9. 填充结果结构体 */
    result->points = best_points;
    result->actual_freq = best_sample_rate / best_points;
    result->actual_sample_rate = best_sample_rate;
    result->frequency_error = fabs(result->actual_freq - target_freq);
    result->error_percent = (result->frequency_error / target_freq) * 100.0;
    result->success = true;  // 成功生成
}

/**
 * @brief  生成并启动正弦波DDS输出 - 封装版本
 * @param  module: DAC80004模块结构体指针
 * @param  target_freq: 目标频率 (Hz)
 * @param  max_sample_rate: 最大采样率 (Hz)
 * @param  min_points: 最小点数
 * @param  max_points: 最大点数
 * @param  amplitude: 幅值 (0-32767)
 * @param  offset: 直流偏移 (0-65535)
 * @param  repeat_count: 重复次数 (0=无限循环)
 * @param  wave_buffer: 临时波形缓冲区
 * @param  wave_high_data: 高16位数据缓冲区
 * @param  wave_low_data: 低16位数据缓冲区
 * @param  result: 输出结果结构体指针
 * @retval true: 成功启动, false: 启动失败
 */
bool Generate_And_Start_Sine_DDS(DAC80004_InitStruct *module,
                                 double target_freq, double max_sample_rate,
                                 uint32_t min_points, uint32_t max_points,
                                 uint16_t amplitude, uint16_t offset,
                                 uint32_t repeat_count,
                                 uint16_t *wave_buffer,
                                 uint16_t *wave_high_data,
                                 uint16_t *wave_low_data)
{
    // 参数验证
    if (module == NULL || wave_buffer == NULL || wave_high_data == NULL || 
        wave_low_data == NULL || &SineWaveResult == NULL) {
        return false;
    }
    
    /* 1. 生成智能正弦波 */
    Generate_Smart_Sine_Wave(wave_buffer, target_freq, max_sample_rate, 
                             min_points, max_points, amplitude, offset, &SineWaveResult);
    
    // 检查正弦波生成是否成功
    if (!SineWaveResult.success) {
        return false;
    }
    
    /* 2. 编码为双DMA格式 */
    Encode_Wave_DualDMA(module, wave_buffer, wave_high_data, wave_low_data, SineWaveResult.points);
    
    /* 3. 启动双DMA传输 */
    bool start_success = DDS_Start_DualDMA(module, WAVE_MODE_SINE, 
                                          wave_high_data, wave_low_data, 
                                          SineWaveResult.points, SineWaveResult.points, 
                                          SineWaveResult.actual_sample_rate, repeat_count);
    
    return start_success;
}

void Sin_Wave_DMA2_Stream5_IRQHandler(void)
{
    /* 检查传输完成中断 */
    if (LL_DMA_IsActiveFlag_TC5(DMA2)) {
        LL_DMA_ClearFlag_TC5(DMA2);
        
        // 只有在有限次数模式下才处理重复逻辑
        if (g_dma_handler.repeat_count > 0) {
            g_dma_handler.stream5.current_count++;  // 增加高16位已完成次数
            
            /* 检查是否需要继续 */
            if (g_dma_handler.stream5.current_count < g_dma_handler.repeat_count) {
                // 还需要继续传输，重新启动Stream5
                LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
                while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}
                
                // 重新设置Stream5参数
                LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)g_dma_handler.stream5.data_ptr);
                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, g_dma_handler.stream5.data_points);
                
                // 重新启动Stream5
                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
            } else {
                // 高16位传输完成
                g_dma_handler.stream5.transfer_complete = 1;  // 设置高16位完成标志
                
                // 检查是否两个流都完成了
                if (g_dma_handler.stream4.transfer_complete == 1) {
                    // 两个流都完成，停止整个传输
                    g_dma_handler.state = DMA_STATE_COMPLETED;
                    DDS_Stop_DualDMA();
                    SYNC_Cycle_Stop();
                }
            }
        }
    }
    
    /* 检查传输错误中断 */
    if (LL_DMA_IsActiveFlag_TE5(DMA2)) {
        LL_DMA_ClearFlag_TE5(DMA2);
        g_dma_handler.state = DMA_STATE_ERROR;
        DDS_Stop_DualDMA();
        LED_OFF();
    }
}

void Sin_Wave_DMA2_Stream4_IRQHandler(void)
{
    /* 检查传输完成中断 */
    if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
        LL_DMA_ClearFlag_TC4(DMA2);
        
        // 只有在有限次数模式下才处理重复逻辑
        if (g_dma_handler.repeat_count > 0) {
            g_dma_handler.stream4.current_count++;  // 增加低16位已完成次数
            
            /* 检查是否需要继续 */
            if (g_dma_handler.stream4.current_count < g_dma_handler.repeat_count) {
                // 还需要继续传输，重新启动Stream4
                LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
                while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4)) {}
                
                // 重新设置Stream4参数
                LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)g_dma_handler.stream4.data_ptr);
                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, g_dma_handler.stream4.data_points);
                
                // 重新启动Stream4
                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
            } else {
                // 低16位传输完成
                g_dma_handler.stream4.transfer_complete = 1;  // 设置低16位完成标志
                
                // 检查是否两个流都完成了
                if (g_dma_handler.stream5.transfer_complete == 1) {
                    // 两个流都完成，停止整个传输
                    g_dma_handler.state = DMA_STATE_COMPLETED;
                    DDS_Stop_DualDMA();
                    SYNC_Cycle_Stop();
                }
            }
        }
    }
    
    /* 检查传输错误中断 */
    if (LL_DMA_IsActiveFlag_TE4(DMA2)) {
        LL_DMA_ClearFlag_TE4(DMA2);
        g_dma_handler.state = DMA_STATE_ERROR;
        DDS_Stop_DualDMA();
        LED_OFF();
    }
}
/**
 * @brief  智能循环伏安法信号生成 - 优化版，优先使用最大点数
 * @param  wave_buffer: 输出缓冲区
 * @param  Initial_E: 初始电位 (mV)
 * @param  Final_E: 终止电位 (mV)
 * @param  Scan_Limit1: 扫描极限1 (mV)
 * @param  Scan_Limit2: 扫描极限2 (mV)
 * @param  Scan_Rate: 扫描速率 (mV/s)
 * @param  max_sample_rate: 系统最大允许采样率 (Hz)
 * @param  min_points: 最小点数要求
 * @param  max_points: 最大点数限制
 * @param  result: 输出结果结构体指针
 * @retval None
 */
void Generate_Smart_CV_Wave(uint16_t *wave_buffer,
                            double Initial_E, double Final_E,
                            double Scan_Limit1, double Scan_Limit2,
                            double Scan_Rate, double max_sample_rate,
                            uint32_t min_points, uint32_t max_points,
                            CvWaveResult_t *result)
{
    // 初始化结果结构体
    if (result != NULL) {
        result->points = 0;
        result->actual_sample_rate = 0.0;
        result->total_duration = 0.0;
        result->actual_scan_rate = 0.0;
        result->scan_rate_error = 0.0;
        result->error_percent = 0.0;
        result->total_cycles = 1;  // 固定为1个周期
        result->initial_points = 0;
        result->cycle_total_points = 0;
        result->final_points = 0;
        result->success = false;
    }
    
    // 参数验证
    if (wave_buffer == NULL || result == NULL || Scan_Rate <= 0 || max_sample_rate <= 0 || 
        min_points == 0 || max_points < min_points) {
        return;
    }
    
    /* 1. 计算各段的电位变化和理论时间 */
    // 起始段：Initial_E → Scan_Limit1
    double initial_voltage_change = fabs(Scan_Limit1 - Initial_E);
    double theoretical_initial_time = initial_voltage_change / Scan_Rate;
    
    // 循环段：Scan_Limit1 → Scan_Limit2 → Scan_Limit1
    double cycle_voltage_change = fabs(Scan_Limit2 - Scan_Limit1);
    double theoretical_cycle_time = (2.0 * cycle_voltage_change) / Scan_Rate;  // 正向+反向
    
    // 结束段：Scan_Limit1 → Final_E
    double final_voltage_change = fabs(Final_E - Scan_Limit1);
    double theoretical_final_time = final_voltage_change / Scan_Rate;
    
    // 理论总时间
    double theoretical_total_time = theoretical_initial_time + theoretical_cycle_time + theoretical_final_time;
    
    /* 2. 优化策略：优先使用最大点数，通过调整采样率来匹配 */
    uint32_t best_points = max_points;  // 从最大点数开始
    double best_sample_rate = 0.0;
    double min_error = 1e9;
    
    /* 3. 从最大点数向下搜索，寻找最佳的采样率和点数组合 */
    for (uint32_t points = max_points; points >= min_points; points -= (max_points - min_points + 1) / 100) {
        // 基于点数计算对应的采样率
        double sample_rate = points / theoretical_total_time;
        
        // 检查采样率是否在允许范围内
        if (sample_rate > max_sample_rate) {
            continue;  // 跳过超出最大采样率的点数
        }
        
        // 计算实际的时间分配
        double actual_total_time = points / sample_rate;
        double time_scale = actual_total_time / theoretical_total_time;
        
        // 计算实际各段时间
        double actual_initial_time = theoretical_initial_time * time_scale;
        double actual_cycle_time = theoretical_cycle_time * time_scale;
        double actual_final_time = theoretical_final_time * time_scale;
        
        // 计算实际扫描速率
        double actual_scan_rate = cycle_voltage_change / (actual_cycle_time / 2.0);
        double scan_rate_error = fabs(actual_scan_rate - Scan_Rate);
        double error_percent = (scan_rate_error / Scan_Rate) * 100.0;
        
        // 寻找误差最小的组合
        if (scan_rate_error < min_error) {
            min_error = scan_rate_error;
            best_points = points;
            best_sample_rate = sample_rate;
            
            // 如果误差小于1%，可以接受
            if (error_percent < 1.0) break;
        }
    }
    
    /* 4. 如果没有找到合适的组合，使用备用策略 */
    if (best_sample_rate == 0.0) {
        // 使用理论计算的采样率，限制在允许范围内
        best_sample_rate = best_points / theoretical_total_time;
        if (best_sample_rate > max_sample_rate) {
            best_sample_rate = max_sample_rate;
            best_points = (uint32_t)round(best_sample_rate * theoretical_total_time);
        }
        if (best_points < min_points) {
            best_points = min_points;
            best_sample_rate = best_points / theoretical_total_time;
        }
    }
    
    /* 5. 基于最终确定的参数计算各段点数分配 */
    double actual_total_time = best_points / best_sample_rate;
    double time_scale = actual_total_time / theoretical_total_time;
    
    // 计算各段实际时间
    double actual_initial_time = theoretical_initial_time * time_scale;
    double actual_cycle_time = theoretical_cycle_time * time_scale;
    double actual_final_time = theoretical_final_time * time_scale;
    
    // 计算各段点数
    uint32_t initial_points = (uint32_t)round(best_sample_rate * actual_initial_time);
    uint32_t cycle_points = (uint32_t)round(best_sample_rate * actual_cycle_time);
    uint32_t final_points = (uint32_t)round(best_sample_rate * actual_final_time);
    
    // 确保总点数准确
    uint32_t calculated_total = initial_points + cycle_points + final_points;
    if (calculated_total != best_points) {
        int32_t diff = best_points - calculated_total;
        // 优先调整循环部分的点数，因为它对精度影响最大
        cycle_points += diff;
    }
    
    // 确保各段点数不为0
    if (initial_points == 0 && initial_voltage_change > 0) initial_points = 1;
    if (cycle_points < 2 && cycle_voltage_change > 0) cycle_points = 2;  // 循环至少需要2个点
    if (final_points == 0 && final_voltage_change > 0) final_points = 1;
    
    /* 6. 设置结果结构体的点数信息 */
    result->initial_points = initial_points;
    result->cycle_total_points = cycle_points;
    result->final_points = final_points;
    
    /* 7. 生成波形数据 */
    uint32_t index = 0;

    // 7.1 起始部分：Initial_E → Scan_Limit1
    for (uint32_t i = 0; i < initial_points && index < best_points; i++, index++) {
        double progress = (initial_points > 1) ? (double)i / (initial_points - 1) : 0.0;
        double voltage = Initial_E + progress * (Scan_Limit1 - Initial_E);
        
        // 转换为DAC值 (电位范围-5000mV到+5000mV映射到0-65535)
        int32_t dac_value = (int32_t)(32768 + (voltage * 32768.0 / 5000.0));
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        wave_buffer[index] = (uint16_t)dac_value;
    }

    // 7.2 循环部分：Scan_Limit1 → Scan_Limit2 → Scan_Limit1
    uint32_t half_cycle_points = cycle_points / 2;

    // 正向扫描：Scan_Limit1 → Scan_Limit2
    for (uint32_t i = 0; i < half_cycle_points && index < best_points; i++, index++) {
        double progress = (half_cycle_points > 1) ? (double)i / (half_cycle_points - 1) : 0.0;
        double voltage = Scan_Limit1 + progress * (Scan_Limit2 - Scan_Limit1);
        
        int32_t dac_value = (int32_t)(32768 + (voltage * 32768.0 / 5000.0));
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        wave_buffer[index] = (uint16_t)dac_value;
    }

    // 反向扫描：Scan_Limit2 → Scan_Limit1
    uint32_t remaining_cycle_points = cycle_points - half_cycle_points;
    for (uint32_t i = 0; i < remaining_cycle_points && index < best_points; i++, index++) {
        double progress = (remaining_cycle_points > 1) ? (double)i / (remaining_cycle_points - 1) : 0.0;
        double voltage = Scan_Limit2 + progress * (Scan_Limit1 - Scan_Limit2);
        
        int32_t dac_value = (int32_t)(32768 + (voltage * 32768.0 / 5000.0));
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        wave_buffer[index] = (uint16_t)dac_value;
    }

    // 7.3 结束部分：Scan_Limit1 → Final_E
    for (uint32_t i = 0; i < final_points && index < best_points; i++, index++) {
        double progress = (final_points > 1) ? (double)i / (final_points - 1) : 0.0;
        double voltage = Scan_Limit1 + progress * (Final_E - Scan_Limit1);
        
        int32_t dac_value = (int32_t)(32768 + (voltage * 32768.0 / 5000.0));
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        wave_buffer[index] = (uint16_t)dac_value;
    }

    /* 8. 填充剩余点（如果有） */
    while (index < best_points) {
        int32_t dac_value = (int32_t)(32768 + (Final_E * 32768.0 / 5000.0));
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        wave_buffer[index] = (uint16_t)dac_value;
        index++;
    }
    
    /* 9. 填充结果结构体 */
    result->points = best_points;
    result->actual_sample_rate = best_sample_rate;
    result->total_duration = best_points / best_sample_rate;
    result->actual_scan_rate = cycle_voltage_change / (cycle_points / best_sample_rate / 2.0);
    result->scan_rate_error = fabs(result->actual_scan_rate - Scan_Rate);
    result->error_percent = (result->scan_rate_error / Scan_Rate) * 100.0;
    result->success = true;
}

/**
 * @brief  生成并启动循环伏安法DDS输出 - 封装版本
 * @param  module: DAC80004模块结构体指针
 * @param  Initial_E: 初始电位 (mV)
 * @param  Final_E: 终止电位 (mV)
 * @param  Scan_Limit1: 扫描极限1 (mV)
 * @param  Scan_Limit2: 扫描极限2 (mV)
 * @param  Scan_Rate: 扫描速率 (mV/s)
 * @param  max_sample_rate: 最大采样率 (Hz)
 * @param  min_points: 最小点数
 * @param  max_points: 最大点数
 * @param  repeat_count: 重复次数 (0=无限循环)
 * @param  wave_buffer: 临时波形缓冲区
 * @param  wave_high_data: 高16位数据缓冲区
 * @param  wave_low_data: 低16位数据缓冲区
 * @retval true: 成功启动, false: 启动失败
 */
bool Generate_And_Start_CV_DDS(DAC80004_InitStruct *module,
                               double Initial_E, double Final_E,
                               double Scan_Limit1, double Scan_Limit2,
                               double Scan_Rate, double max_sample_rate,
                               uint32_t min_points, uint32_t max_points,
                               uint32_t repeat_count,
                               uint16_t *wave_buffer,
                               uint16_t *wave_high_data,
                               uint16_t *wave_low_data)
{
    // 参数验证
    if (module == NULL || wave_buffer == NULL || wave_high_data == NULL || 
        wave_low_data == NULL || &CvWaveResult == NULL) {
        return false;
    }
    
    /* 1. 生成智能循环伏安法波形 */
    Generate_Smart_CV_Wave(wave_buffer, Initial_E, Final_E, Scan_Limit1, Scan_Limit2,
                          Scan_Rate, max_sample_rate, min_points, max_points, &CvWaveResult);
    
    // 检查CV波形生成是否成功
    if (!CvWaveResult.success) {
        return false;
    }
    
    /* 2. 编码为双DMA格式 */
    Encode_Wave_DualDMA(module, wave_buffer, wave_high_data, wave_low_data, CvWaveResult.points);
    
    /* 3. 启动双DMA传输 */
    bool start_success = DDS_Start_DualDMA(module, WAVE_MODE_CV, 
                                          wave_high_data, wave_low_data, 
                                          CvWaveResult.points, CvWaveResult.points, 
                                          CvWaveResult.actual_sample_rate, repeat_count);
    
    return start_success;
}
/**
 * @brief  CV波形DMA2 Stream5中断处理函数
 */
void CV_Wave_DMA2_Stream5_IRQHandler(void)
{
    /* 检查传输完成中断 */
    if (LL_DMA_IsActiveFlag_TC5(DMA2)) {
        LL_DMA_ClearFlag_TC5(DMA2);
        
        // 只有在有限次数模式下才处理重复逻辑
        if (g_dma_handler.repeat_count > 0) {
            g_dma_handler.stream5.current_count++;  // 增加高16位已完成次数
            
            /* 检查是否需要继续 */
            if (g_dma_handler.stream5.current_count < g_dma_handler.repeat_count) {
                // 还需要继续传输，重新启动Stream5
                LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
                while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}
                
                // 重新设置Stream5参数
                LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)g_dma_handler.stream5.data_ptr);
                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, g_dma_handler.stream5.data_points);
                
                // 重新启动Stream5
                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
            } else {
                // 高16位传输完成
                g_dma_handler.stream5.transfer_complete = 1;  // 设置高16位完成标志
                
                // 检查是否两个流都完成了
                if (g_dma_handler.stream4.transfer_complete == 1) {
                    // 两个流都完成，停止整个传输
                    g_dma_handler.state = DMA_STATE_COMPLETED;
                    DDS_Stop_DualDMA();
                    SYNC_Cycle_Stop();
                }
            }
        }
    }
    
    /* 检查传输错误中断 */
    if (LL_DMA_IsActiveFlag_TE5(DMA2)) {
        LL_DMA_ClearFlag_TE5(DMA2);
        g_dma_handler.state = DMA_STATE_ERROR;
        DDS_Stop_DualDMA();
        LED_OFF();
    }
}

/**
 * @brief  CV波形DMA2 Stream4中断处理函数
 */
void CV_Wave_DMA2_Stream4_IRQHandler(void)
{
    /* 检查传输完成中断 */
    if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
        LL_DMA_ClearFlag_TC4(DMA2);
        
        // 只有在有限次数模式下才处理重复逻辑
        if (g_dma_handler.repeat_count > 0) {
            g_dma_handler.stream4.current_count++;  // 增加低16位已完成次数
            
            /* 检查是否需要继续 */
            if (g_dma_handler.stream4.current_count < g_dma_handler.repeat_count) {
                // 还需要继续传输，重新启动Stream4
                LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
                while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4)) {}
                
                // 重新设置Stream4参数
                LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)g_dma_handler.stream4.data_ptr);
                LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, g_dma_handler.stream4.data_points);
                
                // 重新启动Stream4
                LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
            } else {
                // 低16位传输完成
                g_dma_handler.stream4.transfer_complete = 1;  // 设置低16位完成标志
                
                // 检查是否两个流都完成了
                if (g_dma_handler.stream5.transfer_complete == 1) {
                    // 两个流都完成，停止整个传输
                    g_dma_handler.state = DMA_STATE_COMPLETED;
                    DDS_Stop_DualDMA();
                    SYNC_Cycle_Stop();
                }
            }
        }
    }
    
    /* 检查传输错误中断 */
    if (LL_DMA_IsActiveFlag_TE4(DMA2)) {
        LL_DMA_ClearFlag_TE4(DMA2);
        g_dma_handler.state = DMA_STATE_ERROR;
        DDS_Stop_DualDMA();
        LED_OFF();
    }
}







/**
 * @brief  DMA2 Stream5中断处理函数 - 双DMA模式（高16位数据流）
 */
void DMA2_Stream5_IRQHandler(void)
{
    switch (g_dma_handler.current_mode)
    {
    case WAVE_MODE_SINE:
        Sin_Wave_DMA2_Stream5_IRQHandler();
        break;
    case WAVE_MODE_CV:
        CV_Wave_DMA2_Stream5_IRQHandler();
        break;
    default:
        break;
    }
}

/**
 * @brief  DMA2 Stream4中断处理函数 - 双DMA模式（低16位数据流）
 */
void DMA2_Stream4_IRQHandler(void)
{
    switch (g_dma_handler.current_mode)
    {
    case WAVE_MODE_SINE:
        Sin_Wave_DMA2_Stream4_IRQHandler();  // 注意：这里之前有错误，应该是Stream4处理函数
        break;
    case WAVE_MODE_CV:
        CV_Wave_DMA2_Stream4_IRQHandler();
        break;
    default:
        break;
    }
}

/**
 * @brief  波形数据编码 - 双DMA版本，输出分离的数组
 * @param  module: DAC80004模块结构体指针
 * @param  wave_buffer_in: 输入波形数据缓冲区 (16位DAC值)
 * @param  wave_buffer_high_out: 输出高16位数据缓冲区
 * @param  wave_buffer_low_out: 输出低16位数据缓冲区
 * @param  points: 数据点数
 * @retval None
 */
void Encode_Wave_DualDMA(DAC80004_InitStruct *module, uint16_t *wave_buffer_in, 
                        uint16_t *wave_buffer_high_out, uint16_t *wave_buffer_low_out, uint32_t points)
{
    // 参数验证
    if (module == NULL || wave_buffer_in == NULL || wave_buffer_high_out == NULL || 
        wave_buffer_low_out == NULL || points == 0) {
        return;
    }
    
    // 生成DAC80004发送控制位掩码（不包含数据位）
    uint32_t control_mask = module->TX_Data & ~(0xFFFF << 4);
    
    // 编码波形数据为分离数组格式
    for (uint32_t i = 0; i < points; i++) {
        // 将16位DAC数据与控制位组合成32位传输数据
        uint32_t encoded_data = control_mask | ((uint32_t)wave_buffer_in[i] << 4);
        
        // 分离存储到两个独立数组
        wave_buffer_high_out[i] = (uint16_t)(encoded_data >> 16);  // 高16位
        wave_buffer_low_out[i]  = (uint16_t)(encoded_data & 0xFFFF); // 低16位
    }
}