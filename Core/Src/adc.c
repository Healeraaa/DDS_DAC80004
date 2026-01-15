#include "adc.h"
void ADC1_Init(void)
{

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 启用外设时钟 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

  /*
  ADC1 GPIO Configuration
  PC5   ------> ADC1_IN15
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /** 配置 ADC1 的初始化参数 */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;           // 设置 ADC 分辨率为 12 位（常用选择）
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;      // 设置数据对齐方式为右对齐
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE; // 禁用扫描模式，表示只使用一个通道
  LL_ADC_Init(ADC1, &ADC_InitStruct);                          // 将配置应用到 ADC1

  /** 配置 ADC1 的注册相关设置 */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;          // 触发源设置为软件触发
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;     // 禁用序列扫描模式
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // 禁用序列不连续模式
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;           // 设置为单次转换模式
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;        // 禁用 DMA 传输
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);                           // 将配置应用到 ADC1 的转换寄存器

  /** 设置 ADC1 转换完成标志 */
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV); // 设置结束标志为单次转换完成

  /** 配置 ADC 的通用设置 */
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;// 设置 ADC 时钟源为同步时钟，分频系数为 4
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  
  /** 配置 ADC1 的转换序列和通道设置 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_15);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_3CYCLES);

  LL_ADC_Enable(ADC1); 
}

/**
 * @brief 读取 ADC1 通道 1 的值
 * @retval 返回 ADC 转换结果（12 位）
 */
uint16_t ADC1_Read(void)
{
  LL_ADC_REG_StartConversionSWStart(ADC1); // 触发 ADC 转换
  while (!LL_ADC_IsActiveFlag_EOCS(ADC1)){}
  uint16_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1); // 读取 12 位转换结果
  LL_ADC_ClearFlag_EOCS(ADC1); // 清除 EOC 标志
  return adc_value;
}

/**
 * @brief 对ADC采集值进行均值滤波
 * @retval 返回 ADC 转换结果（12 位）
 */
uint16_t ADC1_FifterRead(void)
{
  uint64_t fifter_value = 0;  
  for (uint8_t i = 0; i < 100; i++)
  {
    fifter_value += ADC1_Read();
  }
  fifter_value/=100;
  return fifter_value;
}
