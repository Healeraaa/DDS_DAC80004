#include "DDS_DAC80004.h"
#include "spi.h"
#include "tim.h"
#include "dma.h"
#include <math.h>       


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 全局变量定义
volatile uint32_t g_dds_repeat_count = 0;     // 目标循环次数
volatile uint32_t g_dds_current_count = 0;    // 当前已完成的循环次数
volatile uint16_t *g_wave_data = NULL;        // 波形数据指针
volatile uint16_t g_data_size = 0;            // 单次传输的数据大小
volatile uint8_t g_dds_transfer_complete = 0; // 传输完成标志

void DDS_Init(DAC80004_InitStruct *module)
{
    DAC80004_Init(module);// 初始化GPIO
    MX_DMA_Init();
    TIM1_DMA_SPI1_Init();
}

/**
 * @brief  DDS启动（精确模式）- 支持double精度频率
 * @param  wave_data: 波形数据缓冲区
 * @param  data_size: 数据大小
 * @param  sample_rate: 采样率 (Hz) - 支持double精度小数
 * @retval true: 成功, false: 失败
 */
bool DDS_Start_Precise(uint16_t *wave_data, uint16_t data_size, double sample_rate)
{
    // 等待SPI发送缓冲区为空和SPI不再忙碌
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}  // 确保SPI发送缓冲区空闲
    while(LL_SPI_IsActiveFlag_BSY(SPI1)) {}   // 确保SPI总线不忙碌
    
    /* 1. 安全停止当前DMA传输 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);  // 禁用DMA2流5传输
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}  // 等待DMA完全停止
    
    /* 2. 清除所有DMA中断标志位，确保干净的启动状态 */
    LL_DMA_ClearFlag_TE5(DMA2);   // 清除传输错误标志
    LL_DMA_ClearFlag_TC5(DMA2);   // 清除传输完成标志
    LL_DMA_ClearFlag_DME5(DMA2);  // 清除直接模式错误标志
    LL_DMA_ClearFlag_FE5(DMA2);   // 清除FIFO错误标志
    LL_DMA_ClearFlag_HT5(DMA2);   // 清除半传输完成标志
    
    /* 3. 配置DMA源地址和传输参数 */
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_data);  // 设置波形数据源地址
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, 2*data_size);               // 设置传输数据长度
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);          // 设置循环传输模式
    
    /* 4. 使用精确分频算法计算最优定时器配置 */
    TIM_FreqConfig_t freq_config;  // 定义频率配置结构体
    uint32_t timer_clock = 100000000;  // 定时器基准时钟100MHz（根据系统时钟配置）
    
    // 调用精确分频计算函数，支持double精度频率
    if (TIM_CalculateFreqDivision_Precise(timer_clock, sample_rate, &freq_config) != 0) {
        return false;  
    }
    
    /* 5. 应用计算出的精确定时器配置 */
    TIM_ApplyFreqConfig(TIM1, &freq_config);  // 将最优预分频器和周期值应用到TIM1
    
    /* 7. 使能关键的硬件触发链路 */
    LL_SPI_EnableDMAReq_TX(SPI1);        // 使能SPI1的DMA发送请求
    LL_TIM_EnableDMAReq_UPDATE(TIM1);    // 使能TIM1的UPDATE事件触发DMA

    /* 8. 启动硬件传输链：TIM1 → DMA2 → SPI1 */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);  // 启动DMA流，准备接收定时器触发
    LL_TIM_EnableCounter(TIM1);                  // 启动定时器，开始产生触发信号
    
    return true; 
}

bool DDS_Start_Repeat(DAC80004_InitStruct *module,uint16_t *wave_data, uint16_t data_size, double sample_rate, uint32_t repeat_count)
{
    // 等待SPI空闲
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
    while(LL_SPI_IsActiveFlag_BSY(SPI1)) {}
    
    /* 1. 停止当前DMA传输 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}
    
    /* 2. 清除DMA标志位 */
    LL_DMA_ClearFlag_TE5(DMA2);
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_DME5(DMA2);
    LL_DMA_ClearFlag_FE5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);


    /* 3. 设置循环参数 */
    g_dds_repeat_count = repeat_count;
    g_dds_current_count = 0;
    g_wave_data = wave_data;
    g_data_size = data_size;
    g_dds_transfer_complete = 0;
    
    /* 4. 配置DMA */
    if (repeat_count == 0) {
        // 无限循环模式
        LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_data);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, data_size);
    } else {
        // 有限次数模式 - 使用普通模式，每次传输完成后重新启动
        LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_data);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, data_size);
        
        // 使能DMA传输完成中断
        LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
        LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_5);
        NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    }
    
    /* 5. 配置定时器频率 */
    TIM_FreqConfig_t freq_config;
    uint32_t timer_clock = 100000000;
    
    if (TIM_CalculateFreqDivision_Precise(timer_clock, sample_rate, &freq_config) != 0) {
        return false;
    }
    
    TIM_ApplyFreqConfig(TIM1, &freq_config);
    
    /* 6. 启动传输 */
    DAC8004_CSL_Config(module, 0); // 使能片选
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_TIM_EnableDMAReq_UPDATE(TIM1);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
    LL_TIM_EnableCounter(TIM1);
    
    return true;
}

void DMA2_Stream5_IRQHandler(void)
{
    /* 检查传输完成中断 */
    if (LL_DMA_IsActiveFlag_TC5(DMA2)) {
        LL_DMA_ClearFlag_TC5(DMA2);
        
        g_dds_current_count++;  // 增加已完成次数
        
        /* 检查是否需要继续 */
        if (g_dds_current_count < g_dds_repeat_count) {
            // 还需要继续传输，重新启动DMA
            LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
            while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}
            
            // 重新设置DMA参数
            LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)g_wave_data);
            LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, g_data_size);
            
            // 重新启动DMA
            LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
        } else {
            // 所有循环完成，停止传输
            LL_TIM_DisableCounter(TIM1);
            LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
            LL_SPI_DisableDMAReq_TX(SPI1);
            
            g_dds_transfer_complete = 1;  // 设置完成标志
            

        }
    }
    
    /* 检查传输错误中断 */
    if (LL_DMA_IsActiveFlag_TE5(DMA2)) {
        LL_DMA_ClearFlag_TE5(DMA2);
        DDS_Stop();
        LED_OFF();
    }
}






/**
 * @brief  停止DDS输出
 */
void DDS_Stop(void)
{
    LL_TIM_DisableCounter(TIM1);                // 改为TIM1
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5); // 改为DMA2_STREAM_5
    LL_SPI_DisableDMAReq_TX(SPI1);              // 禁用SPI DMA请求
}

/**
 * @brief  智能正弦波生成 - 通过参数返回结果
 * @param  wave_buffer: 输出缓冲区
 * @param  target_freq: 目标频率 (Hz)
 * @param  max_sample_rate: 最大允许采样率
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
    
    double best_sample_rate = max_sample_rate;
    uint32_t best_points = 0;
    double min_error = 1e9;
    
    // 遍历寻找最优采样率和点数组合
    for (double sr = max_sample_rate; sr >= target_freq * min_points; sr -= target_freq * 0.01) {
        volatile uint32_t points = (uint32_t)round(sr / target_freq);
        // volatile double points_double = sr / target_freq;  // 先计算double值
        // volatile uint32_t points = (uint32_t)round(points_double); 
        
        if (points >= min_points && points <= max_points) {
            double actual_f = sr / points;
            double error = fabs(actual_f - target_freq);
            
            if (error < min_error) {
                min_error = error;
                best_sample_rate = sr;
                best_points = points;
                
                // 如果误差足够小，提前退出
                if (error < target_freq * 1e-6) break;
            }
        }
    }
    
    // 如果没找到合适的，使用默认值
    if (best_points == 0) {
        best_points = (uint32_t)round(max_sample_rate / target_freq);
        if (best_points < min_points) best_points = min_points;
        if (best_points > max_points) best_points = max_points;
        best_sample_rate = target_freq * best_points;
    }
    
    // 生成正弦波数据
    for (uint32_t i = 0; i < best_points; i++) {
        double phase = (2.0 * M_PI * i) / best_points;
        double sine_value = sin(phase);
        
        int32_t dac_value = offset + (int32_t)(amplitude * sine_value);
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 65535) dac_value = 65535;
        
        wave_buffer[i] = (uint16_t)dac_value;
    }
    
    // 填充结果结构体
    result->points = best_points;
    result->actual_freq = best_sample_rate / best_points;
    result->actual_sample_rate = best_sample_rate;
    result->frequency_error = fabs(result->actual_freq - target_freq);
    result->error_percent = (result->frequency_error / target_freq) * 100.0;
    result->success = true;  // 成功生成
}


/**
 * @brief  波形数据编码 - 将DAC数据编码为DAC80004传输格式
 * @param  module: DAC80004模块结构体指针
 * @param  wave_buffer_in: 输入波形数据缓冲区 (16位DAC值)
 * @param  wave_buffer_out: 输出编码后的数据缓冲区 (16位SPI传输格式)
 * @param  points: 数据点数
 * @retval None
 */
void Encode_Wave(DAC80004_InitStruct *module, uint16_t *wave_buffer_in, 
                 uint16_t *wave_buffer_out, uint32_t points)
{
    // 参数验证
    if (module == NULL || wave_buffer_in == NULL || wave_buffer_out == NULL || points == 0) {
        return;
    }
    
    // 生成DAC80004发送控制位掩码（不包含数据位）
    uint32_t control_mask = module->TX_Data & ~(0xFFFF << 4);
    
    // 编码波形数据
    for (uint32_t i = 0; i < points; i++) {
        // 将16位DAC数据与控制位组合成32位传输数据
        uint32_t encoded_data = control_mask | ((uint32_t)wave_buffer_in[i] << 4);
        
        // 分离为两个16位数据用于SPI传输
        wave_buffer_out[2*i]     = (uint16_t)(encoded_data >> 16);  // 高16位
        wave_buffer_out[2*i + 1] = (uint16_t)(encoded_data & 0xFFFF); // 低16位
    }
}