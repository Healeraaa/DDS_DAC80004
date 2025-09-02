#include "DDS_DAC80004.h"

#include "spi.h"
#include "tim.h"
#include "dma.h"
// #include "main_init.h"

void DDS_Init(DAC80004_InitStruct *module)
{
    DAC80004_Init(module);// 初始化GPIO
    // SPI1_Init();
    MX_DMA_Init();
    TIM1_DMA_SPI1_Init();

    
}

/**
 * @brief  设置DDS参数并启动
 * @param  wave_data: 波形数据缓冲区
 * @param  data_size: 数据大小
 * @param  sample_rate: 采样率 (Hz)
 */
// void DDS_Start(uint16_t *wave_data, uint16_t data_size, uint32_t sample_rate)
// {

//     // 等待 SPI 发送缓冲区为空
// while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}

// // 等待 SPI 不再忙碌
// while(LL_SPI_IsActiveFlag_BSY(SPI1)) {}


//         /* 1. 设置DMA内存地址和数据长度 */
//     LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);  // 先停止DMA
//     while(LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_2)){}  // 等待停止完成
    
//     LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_2, (uint32_t)wave_data);
//     LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, data_size);
    
//     /* 2. 计算并设置定时器周期(采样率) */
//     uint32_t timer_clock = 100000000;  // 100MHz系统时钟
//     uint32_t period = (timer_clock / sample_rate) - 1;
//     LL_TIM_SetAutoReload(TIM3, period);
    
//     /* 3. 确保SPI1已经配置好DMA模式 */
//     LL_SPI_EnableDMAReq_TX(SPI1);  // 关键：使能SPI1的DMA发送请求
//     LL_TIM_EnableDMAReq_UPDATE(TIM3);  // 这行代码实现硬件触发！
    
//     /* 6. 启动DMA流 */
//     LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
    
//     /* 7. 启动定时器 - 开始硬件触发链 */
//     LL_TIM_EnableCounter(TIM3);


// }

void DDS_Start(uint16_t *wave_data, uint16_t data_size, uint32_t sample_rate)
{
    // 等待 SPI 发送缓冲区为空
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
    // 等待 SPI 不再忙碌
    while(LL_SPI_IsActiveFlag_BSY(SPI1)) {}
    
    /* 1. 停止DMA传输 - 统一使用DMA2_STREAM_5 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);  // 改为DMA2_STREAM_5
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_5)) {}  // 等待停止完成
    
    /* 2. 清除DMA2 Stream5的所有中断标志 */
    LL_DMA_ClearFlag_TE5(DMA2);   // 传输错误标志
    LL_DMA_ClearFlag_TC5(DMA2);   // 传输完成标志
    LL_DMA_ClearFlag_DME5(DMA2);  // 直接模式错误标志
    LL_DMA_ClearFlag_FE5(DMA2);   // FIFO错误标志
    LL_DMA_ClearFlag_HT5(DMA2);   // 半传输完成标志
    
    /* 3. 设置DMA内存地址和数据长度 - 统一使用DMA2_STREAM_5 */
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)wave_data);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, data_size);
    
    /* 4. 确保DMA配置正确（与TIM1_DMA_SPI1_Init保持一致） */
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);  // 改为循环模式
    
    /* 5. 计算并设置定时器周期(采样率) - 使用TIM1而不是TIM3 */
    uint32_t timer_clock = 100000000;  // 100MHz系统时钟
    uint32_t period = (timer_clock / sample_rate) - 1;
    LL_TIM_SetAutoReload(TIM1, period);  // 改为TIM1
    
    /* 6. 确保关键使能正确 */
    LL_SPI_EnableDMAReq_TX(SPI1);        // 使能SPI1的DMA发送请求
    LL_TIM_EnableDMAReq_UPDATE(TIM1);    // 改为TIM1的DMA请求
    
    /* 7. 启动DMA流 */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);  // 改为DMA2_STREAM_5
    
    /* 8. 启动定时器 */
    LL_TIM_EnableCounter(TIM1);  // 改为TIM1
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