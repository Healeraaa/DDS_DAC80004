#include "tim.h"

void TIM1_DMA_SPI1_Init(void)  // TIM1定时器DMA触发SPI1初始化函数，实现硬件级自动数据传输
{

  LL_TIM_InitTypeDef TIM_InitStruct = {0};  // 定时器初始化结构体

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);  // 使能TIM1外设时钟(APB2总线)

  /* TIM1 DMA Init */

  /* TIM1_UP Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);  // 选择DMA2流5通道6，连接TIM1_UP事件

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);  // 设置DMA传输方向：内存→外设(SPI1)

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);  // 设置DMA最高优先级，确保实时传输

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL); 

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);  // 外设地址不递增(SPI1->DR地址固定)

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);  // 内存地址递增(依次读取波形数据)

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);  // 外设数据宽度16位(匹配SPI1)

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);  // 内存数据宽度16位(匹配波形数据)

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5);  // 禁用FIFO，直接传输模式响应更快

  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI1));  

  /* TIM1基本配置 */
  TIM_InitStruct.Prescaler = 30;  // 预分频器=0，定时器以最高频率运行
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;  // 向上计数模式
  TIM_InitStruct.Autoreload = 65535;  // 自动重装载值(决定采样率)，可调整改变输出频率
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;  // 时钟不分频
  TIM_InitStruct.RepetitionCounter = 0;  // 重复计数器=0(仅高级定时器有此功能)
  LL_TIM_Init(TIM1, &TIM_InitStruct);  // 初始化TIM1定时器
  LL_TIM_DisableARRPreload(TIM1);  // 禁用ARR预装载，寄存器值立即生效
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);  // 设置内部时钟源
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);  // 设置触发输出为更新事件(可用于级联其他定时器)
  LL_TIM_DisableMasterSlaveMode(TIM1);  // 禁用主从模式，独立运行

  LL_TIM_EnableDMAReq_UPDATE(TIM1);  


}