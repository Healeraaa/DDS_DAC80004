#include "ChannelGain.h"

/**
 * @brief  通道增益控制GPIO初始化
 * @note   PB3、PB4控制WE信号通道选择（四选一）
 *         PB5、PB6控制IV转换倍数（四选一：33、1K、10K、100K）
 *         PB7控制第一级电压放大倍数（二选一：1倍、10倍）
 *         PB8、PB9控制第二级电压放大倍数（四选一：1倍、3.3倍、10倍、33倍）
 *         PC10控制第一个反馈选择（二选一：GND、FB）
 *         PC11控制第二个反馈选择（二选一：GND、FB）
 *         PC12控制第三个反馈选择（二选一：GND、FB）
 *         PD2控制第四个反馈选择（二选一：GND、FB）
 */
void ChannelGain_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIO时钟 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

    /* 复位所有输出引脚 */
    LL_GPIO_ResetOutputPin(GPIOB, CHANNEL_GAIN_GPIOB_PINS);
    LL_GPIO_ResetOutputPin(GPIOC, CHANNEL_GAIN_GPIOC_PINS);
    LL_GPIO_ResetOutputPin(GPIOD, CHANNEL_GAIN_GPIOD_PINS);

    /* 配置GPIOB引脚为推挽输出 */
    GPIO_InitStruct.Pin = CHANNEL_GAIN_GPIOB_PINS;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 配置GPIOC引脚为推挽输出 */
    GPIO_InitStruct.Pin = CHANNEL_GAIN_GPIOC_PINS;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 配置GPIOD引脚为推挽输出 */
    GPIO_InitStruct.Pin = CHANNEL_GAIN_GPIOD_PINS;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* 默认设置 */
    WE_Channel_Select(WE_CHANNEL_1);
    IV_Gain_Set(IV_GAIN_33);
    Voltage_Gain_Stage1_Set(VOLTAGE_GAIN_STAGE1_1X);
    Voltage_Gain_Stage2_Set(VOLTAGE_GAIN_STAGE2_1X);
    Feedback1_Select(FEEDBACK_GND);
    Feedback2_Select(FEEDBACK_GND);
    Feedback3_Select(FEEDBACK_GND);
    Feedback4_Select(FEEDBACK_GND);
}

/**
 * @brief  通道增益模块初始化
 */
void ChannelGain_Init(void)
{
    ChannelGain_GPIO_Init();
}

/**
 * @brief  选择WE信号通道
 * @param  channel: WE通道选择
 *         @arg WE_CHANNEL_1: 通道1 (PB4=0, PB3=0)
 *         @arg WE_CHANNEL_2: 通道2 (PB4=0, PB3=1)
 *         @arg WE_CHANNEL_3: 通道3 (PB4=1, PB3=0)
 *         @arg WE_CHANNEL_4: 通道4 (PB4=1, PB3=1)
 */
void WE_Channel_Select(WE_Channel_TypeDef channel)
{
    /* 清除PB3和PB4 */
    LL_GPIO_ResetOutputPin(WE_CHANNEL_GPIO_PORT, WE_CHANNEL_PIN_A | WE_CHANNEL_PIN_B);

    /* 设置PB3 (低位) */
    if (channel & 0x01)
    {
        LL_GPIO_SetOutputPin(WE_CHANNEL_GPIO_PORT, WE_CHANNEL_PIN_A);
    }

    /* 设置PB4 (高位) */
    if (channel & 0x02)
    {
        LL_GPIO_SetOutputPin(WE_CHANNEL_GPIO_PORT, WE_CHANNEL_PIN_B);
    }
}

/**
 * @brief  获取当前WE通道
 * @retval 当前选中的WE通道
 */
WE_Channel_TypeDef WE_Channel_GetCurrent(void)
{
    uint8_t channel = 0;

    if (LL_GPIO_IsOutputPinSet(WE_CHANNEL_GPIO_PORT, WE_CHANNEL_PIN_A))
    {
        channel |= 0x01;
    }
    if (LL_GPIO_IsOutputPinSet(WE_CHANNEL_GPIO_PORT, WE_CHANNEL_PIN_B))
    {
        channel |= 0x02;
    }

    return (WE_Channel_TypeDef)channel;
}

/**
 * @brief  设置IV转换倍数
 * @param  gain: IV转换倍数选择
 *         @arg IV_GAIN_33:   33 (PB6=0, PB5=0)
 *         @arg IV_GAIN_1K:   1K (PB6=0, PB5=1)
 *         @arg IV_GAIN_10K:  10K (PB6=1, PB5=0)
 *         @arg IV_GAIN_100K: 100K (PB6=1, PB5=1)
 */
void IV_Gain_Set(IV_Gain_TypeDef gain)
{
    /* 清除PB5和PB6 */
    LL_GPIO_ResetOutputPin(IV_GAIN_GPIO_PORT, IV_GAIN_PIN_A | IV_GAIN_PIN_B);

    /* 设置PB5 (低位) */
    if (gain & 0x01)
    {
        LL_GPIO_SetOutputPin(IV_GAIN_GPIO_PORT, IV_GAIN_PIN_A);
    }

    /* 设置PB6 (高位) */
    if (gain & 0x02)
    {
        LL_GPIO_SetOutputPin (IV_GAIN_GPIO_PORT, IV_GAIN_PIN_B);
    }
}

/**
 * @brief  获取当前IV转换倍数
 * @retval 当前IV转换倍数设置
 */
IV_Gain_TypeDef IV_Gain_GetCurrent(void)
{
    uint8_t gain = 0;

    if (LL_GPIO_IsOutputPinSet(IV_GAIN_GPIO_PORT, IV_GAIN_PIN_A))
    {
        gain |= 0x01;
    }
    if (LL_GPIO_IsOutputPinSet(IV_GAIN_GPIO_PORT, IV_GAIN_PIN_B))
    {
        gain |= 0x02;
    }

    return (IV_Gain_TypeDef)gain;
}

/**
 * @brief  设置第一级电压放大倍数
 * @param  gain: 第一级电压放大倍数选择
 *         @arg VOLTAGE_GAIN_STAGE1_1X:  1倍 (PB7=0)
 *         @arg VOLTAGE_GAIN_STAGE1_10X: 10倍 (PB7=1)
 */
void Voltage_Gain_Stage1_Set(Voltage_Gain_Stage1_TypeDef gain)
{
    if (gain == VOLTAGE_GAIN_STAGE1_10X)
    {
        LL_GPIO_SetOutputPin(VOLTAGE_GAIN_STAGE1_GPIO_PORT, VOLTAGE_GAIN_STAGE1_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(VOLTAGE_GAIN_STAGE1_GPIO_PORT, VOLTAGE_GAIN_STAGE1_PIN);
    }
}

/**
 * @brief  获取当前第一级电压放大倍数
 * @retval 当前第一级电压放大倍数设置
 */
Voltage_Gain_Stage1_TypeDef Voltage_Gain_Stage1_GetCurrent(void)
{
    if (LL_GPIO_IsOutputPinSet(VOLTAGE_GAIN_STAGE1_GPIO_PORT, VOLTAGE_GAIN_STAGE1_PIN))
    {
        return VOLTAGE_GAIN_STAGE1_10X;
    }
    return VOLTAGE_GAIN_STAGE1_1X;
}

/**
 * @brief  设置第二级电压放大倍数
 * @param  gain: 第二级电压放大倍数选择
 *         @arg VOLTAGE_GAIN_STAGE2_1X:   1倍 (PB9=0, PB8=0)
 *         @arg VOLTAGE_GAIN_STAGE2_3_3X: 3.3倍 (PB9=0, PB8=1)
 *         @arg VOLTAGE_GAIN_STAGE2_10X:  10倍 (PB9=1, PB8=0)
 *         @arg VOLTAGE_GAIN_STAGE2_33X:  33倍 (PB9=1, PB8=1)
 */
void Voltage_Gain_Stage2_Set(Voltage_Gain_Stage2_TypeDef gain)
{
    /* 清除PB8和PB9 */
    LL_GPIO_ResetOutputPin(VOLTAGE_GAIN_STAGE2_GPIO_PORT, VOLTAGE_GAIN_STAGE2_PIN_A | VOLTAGE_GAIN_STAGE2_PIN_B);

    /* 设置PB8 (低位) */
    if (gain & 0x01)
    {
        LL_GPIO_SetOutputPin(VOLTAGE_GAIN_STAGE2_GPIO_PORT, VOLTAGE_GAIN_STAGE2_PIN_A);
    }

    /* 设置PB9 (高位) */
    if (gain & 0x02)
    {
        LL_GPIO_SetOutputPin(VOLTAGE_GAIN_STAGE2_GPIO_PORT, VOLTAGE_GAIN_STAGE2_PIN_B);
    }
}

/**
 * @brief  获取当前第二级电压放大倍数
 * @retval 当前第二级电压放大倍数设置
 */
Voltage_Gain_Stage2_TypeDef Voltage_Gain_Stage2_GetCurrent(void)
{
    uint8_t gain = 0;

    if (LL_GPIO_IsOutputPinSet(VOLTAGE_GAIN_STAGE2_GPIO_PORT, VOLTAGE_GAIN_STAGE2_PIN_A))
    {
        gain |= 0x01;
    }
    if (LL_GPIO_IsOutputPinSet(VOLTAGE_GAIN_STAGE2_GPIO_PORT, VOLTAGE_GAIN_STAGE2_PIN_B))
    {
        gain |= 0x02;
    }

    return (Voltage_Gain_Stage2_TypeDef)gain;
}

/**
 * @brief  设置第一个反馈选择
 * @param  select: 反馈选择
 *         @arg FEEDBACK_GND: 选择GND (PC10=0)
 *         @arg FEEDBACK_FB:  选择FB (PC10=1)
 */
void Feedback1_Select(Feedback_Select_TypeDef select)
{
    if (select == FEEDBACK_FB)
    {
        LL_GPIO_SetOutputPin(FEEDBACK1_GPIO_PORT, FEEDBACK1_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(FEEDBACK1_GPIO_PORT, FEEDBACK1_PIN);
    }
}

/**
 * @brief  获取当前第一个反馈选择
 * @retval 当前反馈选择设置
 */
Feedback_Select_TypeDef Feedback1_GetCurrent(void)
{
    if (LL_GPIO_IsOutputPinSet(FEEDBACK1_GPIO_PORT, FEEDBACK1_PIN))
    {
        return FEEDBACK_FB;
    }
    return FEEDBACK_GND;
}

/**
 * @brief  设置第二个反馈选择
 * @param  select: 反馈选择
 *         @arg FEEDBACK_GND: 选择GND (PC11=0)
 *         @arg FEEDBACK_FB:  选择FB (PC11=1)
 */
void Feedback2_Select(Feedback_Select_TypeDef select)
{
    if (select == FEEDBACK_FB)
    {
        LL_GPIO_SetOutputPin(FEEDBACK2_GPIO_PORT, FEEDBACK2_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(FEEDBACK2_GPIO_PORT, FEEDBACK2_PIN);
    }
}

/**
 * @brief  获取当前第二个反馈选择
 * @retval 当前反馈选择设置
 */
Feedback_Select_TypeDef Feedback2_GetCurrent(void)
{
    if (LL_GPIO_IsOutputPinSet(FEEDBACK2_GPIO_PORT, FEEDBACK2_PIN))
    {
        return FEEDBACK_FB;
    }
    return FEEDBACK_GND;
}

/**
 * @brief  设置第三个反馈选择
 * @param  select: 反馈选择
 *         @arg FEEDBACK_GND: 选择GND (PC12=0)
 *         @arg FEEDBACK_FB:  选择FB (PC12=1)
 */
void Feedback3_Select(Feedback_Select_TypeDef select)
{
    if (select == FEEDBACK_FB)
    {
        LL_GPIO_SetOutputPin(FEEDBACK3_GPIO_PORT, FEEDBACK3_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(FEEDBACK3_GPIO_PORT, FEEDBACK3_PIN);
    }
}

/**
 * @brief  获取当前第三个反馈选择
 * @retval 当前反馈选择设置
 */
Feedback_Select_TypeDef Feedback3_GetCurrent(void)
{
    if (LL_GPIO_IsOutputPinSet(FEEDBACK3_GPIO_PORT, FEEDBACK3_PIN))
    {
        return FEEDBACK_FB;
    }
    return FEEDBACK_GND;
}

/**
 * @brief  设置第四个反馈选择
 * @param  select: 反馈选择
 *         @arg FEEDBACK_GND: 选择GND (PD2=0)
 *         @arg FEEDBACK_FB:  选择FB (PD2=1)
 */
void Feedback4_Select(Feedback_Select_TypeDef select)
{
    if (select == FEEDBACK_FB)
    {
        LL_GPIO_SetOutputPin(FEEDBACK4_GPIO_PORT, FEEDBACK4_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(FEEDBACK4_GPIO_PORT, FEEDBACK4_PIN);
    }
}

/**
 * @brief  获取当前第四个反馈选择
 * @retval 当前反馈选择设置
 */
Feedback_Select_TypeDef Feedback4_GetCurrent(void)
{
    if (LL_GPIO_IsOutputPinSet(FEEDBACK4_GPIO_PORT, FEEDBACK4_PIN))
    {
        return FEEDBACK_FB;
    }
    return FEEDBACK_GND;
}
