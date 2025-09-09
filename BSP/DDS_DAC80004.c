#include "DDS_DAC80004.h"
#include "spi.h"
// #include "tim.h"
#include "dma.h"
#include <math.h>       

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile DMAHandler_t g_dma_handler = {0};

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





// // 修改全局变量定义 - 分离高16位和低16位的管理
// volatile uint32_t g_dual_dma_repeat_count = 0;         // 目标循环次数（共用）
// volatile uint32_t g_dual_dma_high_current_count = 0;   // 高16位当前已完成的循环次数
// volatile uint32_t g_dual_dma_low_current_count = 0;    // 低16位当前已完成的循环次数
// volatile uint8_t g_dual_dma_high_transfer_complete = 0; // 高16位传输完成标志
// volatile uint8_t g_dual_dma_low_transfer_complete = 0;  // 低16位传输完成标志
// volatile uint16_t *g_high_16bit_data = NULL;           // 高16位数据指针
// volatile uint16_t *g_low_16bit_data = NULL;            // 低16位数据指针
// volatile uint16_t g_dual_dma_high_points = 0;          // 高16位数据点数
// volatile uint16_t g_dual_dma_low_points = 0;           // 低16位数据点数



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
    TIM_ApplyFreqConfig_DualDMA(TIM1, &freq_config,100000000,100000000/2);
    SYNC_Cycle_SetPara(&freq_config,100000000,100000000/2); // 设置SYNC信号参数
    
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
 * @brief  DMA2 Stream5中断处理函数 - 双DMA模式（高16位数据流）
 */
void DMA2_Stream5_IRQHandler(void)
{
    Sin_Wave_DMA2_Stream5_IRQHandler();
}

/**
 * @brief  DMA2 Stream4中断处理函数 - 双DMA模式（低16位数据流）
 */
void DMA2_Stream4_IRQHandler(void)
{
    Sin_Wave_DMA2_Stream4_IRQHandler();
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