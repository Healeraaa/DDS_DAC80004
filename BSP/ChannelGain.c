#include "ChannelGain.h"

/**
 * @brief  通道增益控制GPIO初始化
 * @note   PB3、PB4控制WE信号通道选择
 *         PB5控制IV转换增益
 *         PB6、PB7控制电压放大增益
 *         PB8、PB9控制反馈模拟开关通道
 */
void ChannelGain_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_ResetOutputPin(GPIOB, CHANNEL_GAIN_ALL_PINS);
    
    /* 配置PB3, PB4, PB5, PB6, PB7, PB8, PB9为推挽输出 */
    GPIO_InitStruct.Pin = CHANNEL_GAIN_ALL_PINS;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 默认设置：WE通道1，IV增益100，电压增益1倍，FB通道1 */
    WE_Channel_Select(WE_CHANNEL_1);
    IV_Gain_Set(IV_GAIN_100);
    Voltage_Gain_Set(VOLTAGE_GAIN_1);
    FB_Channel_Select(FB_CHANNEL_2);
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
 * @brief  设置IV转换增益
 * @param  gain: IV转换增益选择
 *         @arg IV_GAIN_100:   100挡位 (PB5=0)
 *         @arg IV_GAIN_10000: 10000挡位 (PB5=1)
 */
void IV_Gain_Set(IV_Gain_TypeDef gain)
{
    if (gain == IV_GAIN_10000)
    {
        LL_GPIO_SetOutputPin(IV_GAIN_GPIO_PORT, IV_GAIN_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(IV_GAIN_GPIO_PORT, IV_GAIN_PIN);
    }
}

/**
 * @brief  获取当前IV转换增益
 * @retval 当前IV转换增益设置
 */
IV_Gain_TypeDef IV_Gain_GetCurrent(void)
{
    if (LL_GPIO_IsOutputPinSet(IV_GAIN_GPIO_PORT, IV_GAIN_PIN))
    {
        return IV_GAIN_10000;
    }
    return IV_GAIN_100;
}

/**
 * @brief  设置电压放大增益
 * @param  gain: 电压放大增益选择
 *         @arg VOLTAGE_GAIN_1:    1倍增益 (PB7=0, PB6=0)
 *         @arg VOLTAGE_GAIN_10:   10倍增益 (PB7=0, PB6=1)
 *         @arg VOLTAGE_GAIN_100:  100倍增益 (PB7=1, PB6=0)
 *         @arg VOLTAGE_GAIN_1000: 1000倍增益 (PB7=1, PB6=1)
 */
void Voltage_Gain_Set(Voltage_Gain_TypeDef gain)
{
    /* 清除PB6和PB7 */
    LL_GPIO_ResetOutputPin(VOLTAGE_GAIN_GPIO_PORT, VOLTAGE_GAIN_PIN_A | VOLTAGE_GAIN_PIN_B);
    
    /* 设置PB6 (低位) */
    if (gain & 0x01)
    {
        LL_GPIO_SetOutputPin(VOLTAGE_GAIN_GPIO_PORT, VOLTAGE_GAIN_PIN_A);
    }
    
    /* 设置PB7 (高位) */
    if (gain & 0x02)
    {
        LL_GPIO_SetOutputPin(VOLTAGE_GAIN_GPIO_PORT, VOLTAGE_GAIN_PIN_B);
    }
}

/**
 * @brief  获取当前电压放大增益
 * @retval 当前电压放大增益设置
 */
Voltage_Gain_TypeDef Voltage_Gain_GetCurrent(void)
{
    uint8_t gain = 0;
    
    if (LL_GPIO_IsOutputPinSet(VOLTAGE_GAIN_GPIO_PORT, VOLTAGE_GAIN_PIN_A))
    {
        gain |= 0x01;
    }
    if (LL_GPIO_IsOutputPinSet(VOLTAGE_GAIN_GPIO_PORT, VOLTAGE_GAIN_PIN_B))
    {
        gain |= 0x02;
    }
    
    return (Voltage_Gain_TypeDef)gain;
}

/**
 * @brief  选择反馈模拟开关通道
 * @param  channel: FB通道选择
 *         @arg FB_CHANNEL_1: 通道1 (PB9=0, PB8=0)
 *         @arg FB_CHANNEL_2: 通道2 (PB9=0, PB8=1)
 *         @arg FB_CHANNEL_3: 通道3 (PB9=1, PB8=0)
 *         @arg FB_CHANNEL_4: 通道4 (PB9=1, PB8=1)
 */
void FB_Channel_Select(FB_Channel_TypeDef channel)
{
    /* 清除PB8和PB9 */
    LL_GPIO_ResetOutputPin(FB_CHANNEL_GPIO_PORT, FB_CHANNEL_PIN_A | FB_CHANNEL_PIN_B);
    
    /* 设置PB8 (低位) */
    if (channel & 0x01)
    {
        LL_GPIO_SetOutputPin(FB_CHANNEL_GPIO_PORT, FB_CHANNEL_PIN_A);
    }
    
    /* 设置PB9 (高位) */
    if (channel & 0x02)
    {
        LL_GPIO_SetOutputPin(FB_CHANNEL_GPIO_PORT, FB_CHANNEL_PIN_B);
    }
}

/**
 * @brief  获取当前反馈通道
 * @retval 当前选中的FB通道
 */
FB_Channel_TypeDef FB_Channel_GetCurrent(void)
{
    uint8_t channel = 0;
    
    if (LL_GPIO_IsOutputPinSet(FB_CHANNEL_GPIO_PORT, FB_CHANNEL_PIN_A))
    {
        channel |= 0x01;
    }
    if (LL_GPIO_IsOutputPinSet(FB_CHANNEL_GPIO_PORT, FB_CHANNEL_PIN_B))
    {
        channel |= 0x02;
    }
    
    return (FB_Channel_TypeDef)channel;
}