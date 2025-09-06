#include "tim.h"
#include "float.h"
#include "math.h"

// void TIM1_DMA_SPI1_Init(void) // TIM1定时器DMA触发SPI1初始化函数，实现硬件级自动数据传输
// {

//   LL_TIM_InitTypeDef TIM_InitStruct = {0}; // 定时器初始化结构体

//   /* Peripheral clock enable */
//   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1); // 使能TIM1外设时钟(APB2总线)

//   /* TIM1 DMA Init */

//   /* TIM1_UP Init */
//   LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6); // 选择DMA2流5通道6，连接TIM1_UP事件
//   LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // 设置DMA传输方向：内存→外设(SPI1)
//   LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH); // 设置DMA最高优先级，确保实时传输
//   LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
//   LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT); // 外设地址不递增(SPI1->DR地址固定)
//   LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT); // 内存地址递增(依次读取波形数据)
//   LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD); // 外设数据宽度16位(匹配SPI1)
//   LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD); // 内存数据宽度16位(匹配波形数据)
//   LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5); // 禁用FIFO，直接传输模式响应更快
//   LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI1));

//   /* TIM1基本配置 */
//   TIM_InitStruct.Prescaler = 0;                             // 预分频器=0，定时器以最高频率运行
//   TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;       // 向上计数模式
//   TIM_InitStruct.Autoreload = 65535;                        // 自动重装载值(决定采样率)，可调整改变输出频率
//   TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; // 时钟不分频
//   TIM_InitStruct.RepetitionCounter = 0;                     // 重复计数器=0(仅高级定时器有此功能)
//   LL_TIM_Init(TIM1, &TIM_InitStruct);                       // 初始化TIM1定时器
//   LL_TIM_DisableARRPreload(TIM1);                           // 禁用ARR预装载，寄存器值立即生效
//   LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL); // 设置内部时钟源
//   LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);        // 设置触发输出为更新事件(可用于级联其他定时器)
//   LL_TIM_DisableMasterSlaveMode(TIM1);                      // 禁用主从模式，独立运行

//   LL_TIM_EnableDMAReq_UPDATE(TIM1);
// }


// void TIM1_DMA_SPI1_Init(void)
// {
//   LL_TIM_InitTypeDef TIM_InitStruct = {0};
//   LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  

//   /* 使能TIM1外设时钟 */
//   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

//   /* 配置第一个DMA流：TIM1_UP → DMA2_Stream5 → SPI1 */
//   LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);
//   LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//   LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);
//   LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
//   LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
//   LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
//   LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);
//   LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);
//   LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5);
//   LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI1));

//   /* 修正第二个DMA流配置：TIM1_CC1 → DMA2_Stream4 → SPI1 */
//   /* 注意：TIM1_CC1对应的是DMA2_Stream4_Channel6，不是Channel2 */
//   LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_4, LL_DMA_CHANNEL_6);  // 关键修正：使用Channel6
//   LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//   LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_HIGH);
//   LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
//   LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
//   LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);
//   LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD);
//   LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_HALFWORD);
//   LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_4);
//   LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_4, LL_SPI_DMA_GetRegAddr(SPI1));

//   /* TIM1基本配置 */
//   TIM_InitStruct.Prescaler = 0;
//   TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//   TIM_InitStruct.Autoreload = 65535;  // 将通过函数动态设置
//   TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//   TIM_InitStruct.RepetitionCounter = 0;
//   LL_TIM_Init(TIM1, &TIM_InitStruct);

//   /* 关键修正：配置TIM1比较通道1 */
//   TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;             // 改为PWM模式，确保产生事件
//   TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;         // 使能输出（重要！）
//   TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;       // 互补输出禁用
//   TIM_OC_InitStruct.CompareValue = 32767;                    // 比较值设为ARR的一半
//   TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;     // 极性
//   TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;    // 空闲状态
//   TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;   // 互补空闲状态
//   LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

//   /* 关键：使能比较通道预装载 */
//   LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

//   /* 配置定时器参数 */
//   LL_TIM_DisableARRPreload(TIM1);
//   LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
//   LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
//   LL_TIM_DisableMasterSlaveMode(TIM1);

//   /* 关键：使能DMA请求 */
//   LL_TIM_EnableDMAReq_UPDATE(TIM1);  // 使能UPDATE事件的DMA请求
//   LL_TIM_EnableDMAReq_CC1(TIM1);     // 使能CC1事件的DMA请求
  
//   /* 关键：使能比较通道输出 */
//   LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
// }


// void TIM1_DMA_SPI1_Init(void)
// {
//   LL_TIM_InitTypeDef TIM_InitStruct = {0};
//   LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
//   LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

//   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* 使能TIM1外设时钟 */
//   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

//   /* 配置第一个DMA流：TIM1_UP → DMA2_Stream5 → SPI1 */
//   LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);
//   LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//   LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);
//   LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
//   LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
//   LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
//   LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);
//   LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);
//   LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5);
//   LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI1));

//   /* 注意：TIM1_CC1对应的是DMA2_Stream4_Channel6，不是Channel2 */
//   LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_4, LL_DMA_CHANNEL_6);  // 关键修正：使用Channel6
//   LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//   LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_HIGH);
//   LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
//   LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
//   LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);
//   LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD);
//   LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_HALFWORD);
//   LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_4);
//   LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_4, LL_SPI_DMA_GetRegAddr(SPI1));

//   /* TIM1基本配置 */
//   TIM_InitStruct.Prescaler = 0;
//   TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//   TIM_InitStruct.Autoreload = 65535;  // 将通过函数动态设置
//   TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//   TIM_InitStruct.RepetitionCounter = 0;
//   LL_TIM_Init(TIM1, &TIM_InitStruct);
//   LL_TIM_DisableARRPreload(TIM1);
//   LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

//   /* 关键修正：配置TIM1比较通道1 */
//   TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;             
//   TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;         
//   TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;       // 互补输出禁用
//   TIM_OC_InitStruct.CompareValue = 32767;                    // 比较值设为ARR的一半
//   TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;     // 极性
//   TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;    // 空闲状态
//   TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;   // 互补空闲状态
//   LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
//   LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
//   LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
//   LL_TIM_DisableMasterSlaveMode(TIM1);

//   TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
//   TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
//   TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
//   TIM_BDTRInitStruct.DeadTime = 0;
//   TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
//   TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
//   TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
//   LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

//   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//     /**TIM1 GPIO Configuration
//     PA11     ------> TIM1_CH4
//     */
//   GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
//   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//   GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* 关键：使能比较通道预装载 */
//   LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);



//   /* 关键：使能DMA请求 */
//   LL_TIM_EnableDMAReq_UPDATE(TIM1);  // 使能UPDATE事件的DMA请求
//   LL_TIM_EnableDMAReq_CC4(TIM1);     // 使能CC1事件的DMA请求
  
//   // /* 关键：使能比较通道输出 */
//   // LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
// }
/**
 * @brief 初始化TIM1定时器和DMA，用于通过SPI1进行双事件触发的数据传输
 * @note 此配置使用TIM1的更新事件和通道4比较事件分别触发两个DMA流
 *       更新事件用于第一次传输，通道4比较事件用于第二次传输
 */
void TIM1_DMA_SPI1_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能TIM1外设时钟 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* ======================== 配置第一个DMA流：TIM1_UP → DMA2_Stream5 → SPI1 ======================== */
  /* 此DMA流处理TIM1的更新事件，用于第一次数据传输 */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6); // TIM1_UP事件映射到DMA2流5通道6
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // 内存到外设传输
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH); // 最高优先级
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL); // 正常模式（非循环）
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT); // 外设地址不递增
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT); // 内存地址递增
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD); // 外设数据宽度16位
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD); // 内存数据宽度16位
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5); // 禁用FIFO模式
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI1)); // 设置外设地址为SPI1数据寄存器

  /* ======================== 配置第二个DMA流：TIM1_CH4 → DMA2_Stream4 → SPI1 ======================== */
  /* 此DMA流处理TIM1通道4的比较事件，用于第二次数据传输 */
  /* 注意：TIM1_CH4事件映射到DMA2_Stream4_Channel6 */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_4, LL_DMA_CHANNEL_6);  // 关键：使用通道6
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // 内存到外设传输
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_HIGH); // 高优先级
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL); // 正常模式（非循环）
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT); // 外设地址不递增
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT); // 内存地址递增
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD); // 外设数据宽度16位
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_HALFWORD); // 内存数据宽度16位
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_4); // 禁用FIFO模式
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_4, LL_SPI_DMA_GetRegAddr(SPI1)); // 设置外设地址为SPI1数据寄存器

  /* ======================== TIM1基本配置 ======================== */
  TIM_InitStruct.Prescaler = 0;                     // 预分频器为0（不分频）
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP; // 向上计数模式
  TIM_InitStruct.Autoreload = 65535;                // 自动重装载值（将在运行时动态调整）
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; // 时钟不分频
  TIM_InitStruct.RepetitionCounter = 0;             // 重复计数器为0
  LL_TIM_Init(TIM1, &TIM_InitStruct);               // 应用TIM1配置
  LL_TIM_DisableARRPreload(TIM1);                   // 禁用ARR预装载（更改立即生效）
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL); // 使用内部时钟源

  /* ======================== 配置TIM1通道4输出比较 ======================== */
  /* 配置通道4用于产生比较事件，触发第二次DMA传输 */
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;           // 切换模式（比较匹配时输出翻转）
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;        // 禁用主输出（不产生实际电平输出）
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;       // 禁用互补输出
  TIM_OC_InitStruct.CompareValue = 32767;                    // 比较值设为ARR的一半
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;     // 输出极性为高
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;    // 空闲状态下输出低电平
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;   // 互补输出空闲状态下输出低电平
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct); // 应用配置到TIM1通道4
  
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);  // 禁用快速模式
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);  // 设置触发输出为复位
  LL_TIM_DisableMasterSlaveMode(TIM1);               // 禁用主从模式

  /* ======================== 配置TIM1刹车和死区时间 ======================== */
  /* 高级定时器特有的刹车和死区时间配置 */
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;        // 禁用运行模式下关闭状态
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;        // 禁用空闲模式下关闭状态
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;       // 锁定级别关闭
  TIM_BDTRInitStruct.DeadTime = 0;                           // 死区时间为0
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;      // 禁用刹车输入
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH; // 刹车输入极性为高
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE; // 禁用自动输出
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);               // 应用刹车和死区时间配置

  /* ======================== 配置GPIO ======================== */
  /* 配置PA11为TIM1_CH4的复用功能，即使输出被禁用，仍需要配置GPIO */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);       // 使能GPIOA时钟
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;                      // 选择引脚11
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;             // 设置为复用功能模式
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;            // 设置输出速度为低速
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;      // 设置输出类型为推挽
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;                    // 不使用上拉/下拉电阻
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;                  // 设置复用功能为AF1（TIM1）
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);                     // 应用GPIO配置

  /* ======================== 关键配置 ======================== */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);         // 使能通道4预装载

  /* 使能DMA请求 - 关键修正：使用CC4而不是CC1 */
  LL_TIM_EnableDMAReq_UPDATE(TIM1);  // 使能更新事件的DMA请求（触发第一个DMA流）
  LL_TIM_EnableDMAReq_CC4(TIM1);     // 使能通道4比较事件的DMA请求（触发第二个DMA流）- 关键修正
  
  /* 注意：虽然输出被禁用，但仍需要使能通道以产生比较事件 */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
  
  /* 对于高级定时器TIM1，需要使能主输出 */
  LL_TIM_EnableAllOutputs(TIM1);
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

/**
 * @brief  应用定时器频率配置到指定定时器（双DMA版本）
 * @param  TIMx: 定时器实例
 * @param  config: 频率配置结构体指针
 * @retval None
 */
void TIM_ApplyFreqConfig_DualDMA(TIM_TypeDef *TIMx, const TIM_FreqConfig_t *config)
{
  if (config == NULL)
    return;

  LL_TIM_DisableCounter(TIMx);
  LL_TIM_SetPrescaler(TIMx, config->prescaler);
  LL_TIM_SetAutoReload(TIMx, config->period);
  
  /* 设置CC1比较值为周期的一半，确保在UPDATE和下次UPDATE之间触发 */
  uint32_t cc1_value = (config->period + 1) / 2;  // 周期中点
  LL_TIM_OC_SetCompareCH4(TIMx, cc1_value);
  
  /* 关键：重新使能比较通道 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
}