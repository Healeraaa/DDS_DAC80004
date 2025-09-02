#include "tim.h"
#include "float.h"
#include "math.h"

void TIM1_DMA_SPI1_Init(void) // TIM1定时器DMA触发SPI1初始化函数，实现硬件级自动数据传输
{

  LL_TIM_InitTypeDef TIM_InitStruct = {0}; // 定时器初始化结构体

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1); // 使能TIM1外设时钟(APB2总线)

  /* TIM1 DMA Init */

  /* TIM1_UP Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6); // 选择DMA2流5通道6，连接TIM1_UP事件
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // 设置DMA传输方向：内存→外设(SPI1)
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH); // 设置DMA最高优先级，确保实时传输
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT); // 外设地址不递增(SPI1->DR地址固定)
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT); // 内存地址递增(依次读取波形数据)
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD); // 外设数据宽度16位(匹配SPI1)
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD); // 内存数据宽度16位(匹配波形数据)
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5); // 禁用FIFO，直接传输模式响应更快
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI1));

  /* TIM1基本配置 */
  TIM_InitStruct.Prescaler = 0;                             // 预分频器=0，定时器以最高频率运行
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;       // 向上计数模式
  TIM_InitStruct.Autoreload = 65535;                        // 自动重装载值(决定采样率)，可调整改变输出频率
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; // 时钟不分频
  TIM_InitStruct.RepetitionCounter = 0;                     // 重复计数器=0(仅高级定时器有此功能)
  LL_TIM_Init(TIM1, &TIM_InitStruct);                       // 初始化TIM1定时器
  LL_TIM_DisableARRPreload(TIM1);                           // 禁用ARR预装载，寄存器值立即生效
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL); // 设置内部时钟源
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);        // 设置触发输出为更新事件(可用于级联其他定时器)
  LL_TIM_DisableMasterSlaveMode(TIM1);                      // 禁用主从模式，独立运行

  LL_TIM_EnableDMAReq_UPDATE(TIM1);
}

/**
 * @brief  精确计算定时器分频参数（遍历算法，误差最小）- 支持浮点频率
 * @param  timer_clock: 定时器时钟频率 (Hz)
 * @param  target_freq: 目标频率 (Hz) - 支持小数频率
 * @param  config: 输出的配置结构体指针
 * @retval 0: 成功, 1: 频率超出范围
 */
uint8_t TIM_CalculateFreqDivision_Precise(uint32_t timer_clock, double target_freq, TIM_FreqConfig_t *config)
{
  double min_error_double = DBL_MAX; // 初始化最小误差为double类型的最大值（需要#include "float.h"）

  if (config == NULL || target_freq <= 0.0) // 参数有效性检查：配置指针非空且目标频率大于0
  {
    return 1; // 参数无效，返回错误代码1
  }

  config->error = UINT32_MAX; // 初始化误差为32位无符号整数最大值，用于寻找最小误差
  config->prescaler = 0;      // 初始化预分频器为0（实际分频比=1）
  config->period = 0;         // 初始化自动重装载值为0
  config->actual_freq = 0;    // 初始化实际输出频率为0

  // 遍历所有可能的预分频器值
  for (uint32_t psc = 0; psc <= 65535; psc++) // 遍历16位预分频器的所有可能值(0-65535)，实际分频比为(psc+1)
  {
    double effective_clock = (double)timer_clock / (double)(psc + 1); // 计算预分频后的有效时钟频率，使用double精度避免整数截断
    double calc_period_double = effective_clock / target_freq;        // 计算达到目标频率所需的理论计数周期（浮点数）
    uint32_t calc_period = (uint32_t)round(calc_period_double);       // 四舍五入到最接近的整数周期

    if (calc_period == 0 || calc_period > 65536) // 检查计算的周期是否在16位定时器有效范围内（1到65536）
    {
      continue; // 超出硬件范围，跳过此预分频器值，继续下一次循环
    }

    calc_period -= 1; // 转换为ARR寄存器值：ARR = 计数周期-1（因为定时器从0开始计数到ARR）

    // 计算实际频率和误差(使用double精度提高计算精度)
    double actual_freq_double = effective_clock / (double)(calc_period + 1); // 根据实际ARR值计算真实输出频率
    double error_double = fabs(actual_freq_double - target_freq);            // 计算频率误差的绝对值（fabs用于double类型）
    uint32_t actual_freq_int = (uint32_t)round(actual_freq_double);          // 转换为整数频率用于结构体返回
    uint32_t error_int = (uint32_t)round(error_double);                      // 转换为整数误差用于结构体返回

    // 如果找到更小的误差，更新最优配置参数
    if (error_double < min_error_double) // 使用double精度误差比较，比整数比较更精确
    {
      min_error_double = error_double;       // 更新记录的最小double精度误差
      config->error = error_int;             // 更新整数形式误差供外部使用
      config->prescaler = psc;               // 更新最优预分频器值
      config->period = calc_period;          // 更新最优自动重装载值(ARR)
      config->actual_freq = actual_freq_int; // 更新实际输出频率(整数形式)

      // 如果误差足够小(达到double精度极限)，直接使用当前配置
      if (error_double < (target_freq * 1e-9)) // 误差小于目标频率的十亿分之一时认为已达到最优
        break;                                 // 提前退出循环，避免不必要的计算
    }

    // 性能优化：如果误差已经很小且预分频器较小，优先选择
    if (error_double <= (target_freq * 1e-6) && psc < 1000) // 误差小于0.0001%且预分频器小于1000
    {
      break; // 提前结束搜索，优先选择较小的预分频器以减少计算延迟
    }
  }

  return (min_error_double == DBL_MAX) ? 1 : 0; // 返回计算结果：误差仍为最大值表示未找到有效配置
}

/**
 * @brief  应用定时器频率配置到指定定时器
 * @param  TIMx: 定时器实例 (如TIM1, TIM2等)
 * @param  config: 频率配置结构体指针
 * @retval None
 */
void TIM_ApplyFreqConfig(TIM_TypeDef *TIMx, const TIM_FreqConfig_t *config)
{
  if (config == NULL)
    return;

  LL_TIM_DisableCounter(TIMx);                  // 停止定时器以安全更改配置
  LL_TIM_SetPrescaler(TIMx, config->prescaler); // 设置预分频器
  LL_TIM_SetAutoReload(TIMx, config->period);   // 设置自动重装载值
}