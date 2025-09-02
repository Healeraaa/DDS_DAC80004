#include "spi.h"

volatile uint8_t dma_tx_complete = 0;

/**
 * @brief  SPI1初始化函数
 * @note   配置SPI1为主机模式，16位数据宽度，带DMA传输功能
 *         GPIO配置: PA5(SCK), PA7(MOSI)
 *         DMA配置: DMA2_Stream2用于SPI1_TX
 * @param  None
 * @retval None
 */
void SPI1_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能SPI1和GPIOA时钟 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK   (串行时钟)
  PA7   ------> SPI1_MOSI  (主机输出从机输入)
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;        // 复用功能模式
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; // 超高速
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; // 推挽输出
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;               // 无上下拉
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;             // 复用功能5
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // /* SPI1 DMA初始化 - 配置DMA2_Stream2用于SPI1_TX */
  // LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_2);         // 选择通道2
  // LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // 内存到外设
  // LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);  // 低优先级
  // LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);                  // 正常模式(非循环)
  // LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);  // 外设地址不递增
  // LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);    // 内存地址递增
  // LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);    // 外设数据宽度16位
  // LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);    // 内存数据宽度16位
  // LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);                              // 禁用FIFO模式

  /* SPI1参数配置 */
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;           // 全双工模式
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;                       // 主机模式
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;              // 16位数据宽度
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;             // 时钟空闲时为低电平
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;                 // 第一个边沿采样
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;                           // 软件控制NSS
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128;      // 波特率分频128
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;                     // MSB先发送
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;  // 禁用CRC校验
  SPI_InitStruct.CRCPoly = 10;                                    // CRC多项式(未使用)
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);            // 使用Motorola SPI协议

  // /* 使能SPI1的DMA发送请求 */
  // LL_SPI_EnableDMAReq_TX(SPI1);

  /* 使能SPI1 */
  LL_SPI_Enable(SPI1);
}

void SPI1_Transmit8_Time(uint8_t data, uint32_t Timeout)
{
  uint32_t tickstart = HAL_GetTick(); // 获取当前时间戳

  // 等待发送缓冲区为空（可以写入新数据）
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    // 检查是否超时
    if ((HAL_GetTick() - tickstart) >= Timeout)
    {
      // 超时退出
      return; // 可以根据需要处理超时错误
    }
  }

  // 发送数据
  LL_SPI_TransmitData8(SPI1, data);

  // 等待数据从发送缓冲区转移到移位寄存器
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    // 检查是否超时
    if ((HAL_GetTick() - tickstart) >= Timeout)
    {
      // 超时退出
      return; // 可以根据需要处理超时错误
    }
  }

  // 等待 SPI 总线空闲（数据从移位寄存器发送完成）
  while (LL_SPI_IsActiveFlag_BSY(SPI1))
  {
    // 检查是否超时
    if ((HAL_GetTick() - tickstart) >= Timeout)
    {
      // 超时退出
      return; // 可以根据需要处理超时错误
    }
  }
}

void SPI1_Transmit16_Time(uint16_t data, uint32_t Timeout)
{
  uint32_t tickstart = HAL_GetTick(); // 获取当前时间戳

  // 等待发送缓冲区为空（可以写入新数据）
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    // 检查是否超时
    if ((HAL_GetTick() - tickstart) >= Timeout)
    {
      // 超时退出
      return; // 可以根据需要处理超时错误
    }
  }

  // 发送数据
  LL_SPI_TransmitData16(SPI1, data);

  // 等待数据从发送缓冲区转移到移位寄存器
  while (!LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    // 检查是否超时
    if ((HAL_GetTick() - tickstart) >= Timeout)
    {
      // 超时退出
      return; // 可以根据需要处理超时错误
    }
  }

  // 等待 SPI 总线空闲（数据从移位寄存器发送完成）
  while (LL_SPI_IsActiveFlag_BSY(SPI1))
  {
    // 检查是否超时
    if ((HAL_GetTick() - tickstart) >= Timeout)
    {
      // 超时退出
      return; // 可以根据需要处理超时错误
    }
  }
}


/**
 * @brief  SPI1 数据位宽转换
 * @param  dataWidth This parameter can be one of the following values:
          @arg @ref LL_SPI_DATAWIDTH_8BIT
          @arg @ref LL_SPI_DATAWIDTH_16BIT
 * @retval None
 */
void SPI1_SetDataWidth(uint32_t dataWidth)
{
  // 禁用 SPI，准备更改配置
  LL_SPI_Disable(SPI1);
  // 设置新的数据位宽（8位或16位）
  LL_SPI_SetDataWidth(SPI1, dataWidth);
  // 启用 SPI
  LL_SPI_Enable(SPI1);
}

void SPI1_Transmit_DMA(uint8_t *data, uint32_t size)
{
  while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_3))
  {
  }

  LL_DMA_ClearFlag_TC3(DMA2); // 清除传输完成标志

  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);

  // 设置DMA的外设地址为SPI1的数据寄存器地址
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_3, LL_SPI_DMA_GetRegAddr(SPI1));

  // 设置DMA的内存地址为要传输的数据的起始地址
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_3, (uint32_t)data);

  // 设置DMA传输的数据长度
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, size);

  // 使能DMA流
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);

  // 等待DMA传输完成（可选）
  while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_3))
  {
    // 等待传输完成 
  }
  while (!LL_DMA_IsActiveFlag_TC3(DMA2))
  {
    /* code */
  }

  LL_DMA_ClearFlag_TC3(DMA2); // 清除传输完成标志
}





void SPI1_Transmit_DMA_Start(uint8_t *data, uint32_t size)
{
  dma_tx_complete = 0;

  // 禁用DMA流
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
  LL_DMA_ClearFlag_TC3(DMA2);

  // 配置DMA地址和长度
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_3, LL_SPI_DMA_GetRegAddr(SPI1));
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_3, (uint32_t)data);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, size);

  // 使能DMA传输完成中断
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_3);

  // 使能DMA流，启动传输
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
}

// DMA中断回调函数
void DMA2_Stream3_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC3(DMA2))
  {
    LL_DMA_ClearFlag_TC3(DMA2);
    dma_tx_complete = 1;
    // 关闭DMA传输完成中断（如果不需要连续传输）
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_3);
  }
}


uint8_t SPI1_Transmit_DMA_WaitComplete(void)
{
	if (dma_tx_complete == 1) // 如果标志位为1
	{
		dma_tx_complete = 0;
		return 1; // 则返回1，并自动清零标志位
	}
	return 0; // 如果标志位为0，则返回0
}
