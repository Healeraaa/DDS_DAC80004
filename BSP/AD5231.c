#include "AD5231.h"
#include "spi.h"

/**
 * @brief  AD5231 片选控制
 * @param  state: 1=取消片选(CS高), 0=选中(CS低)
 */
void AD5231_CS(uint8_t state)
{
    if (state) {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
    } else {
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
    }
}

/**
 * @brief  AD5231 GPIO初始化
 */                   
void AD5231_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能GPIOB时钟
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    // 配置CS引脚 (PB12)
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;  // 上拉，默认取消片选
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 初始化SPI2
    SPI2_Init();

    // 默认取消片选
    AD5231_CS(1);
    
    // 延时等待芯片稳定
    LL_mDelay(10);
}

/**
 * @brief  发送命令和数据到AD5231
 * @param  command: 命令字节
 * @param  data: 数据字节 (对于10位数据，使用低10位)
 * @retval AD5231_Status_t
 */
static AD5231_Status_t AD5231_SendCommand(uint8_t command, uint16_t data)
{
    uint8_t cmd_byte0, data_byte1, data_byte0;
    
    // 构建命令字节0 (C3-C0在高4位)
    cmd_byte0 = (command & 0x0F) << 4;
    
    // 构建数据字节1和字节0
    data_byte1 = (data >> 8) & 0xFF;  // 高8位
    data_byte0 = data & 0xFF;          // 低8位
    
    // 片选拉低
    AD5231_CS(0);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    
    // 发送命令字节0
    SPI_Transmit8_Time(SPI2, cmd_byte0, 1);
    
    // 发送数据字节1
    SPI_Transmit8_Time(SPI2, data_byte1, 1);
    
    // 发送数据字节0
    SPI_Transmit8_Time(SPI2, data_byte0, 1);
    
    // 片选拉高
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    AD5231_CS(1);
    
    return AD5231_OK;
}


/**
 * @brief  写入RDAC位置值
 * @param  position: 位置值 (0-1023)
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_WriteRDAC(uint16_t position)
{
    if (position > AD5231_MAX_POSITION) {
        return AD5231_ERROR_INVALID_POSITION;
    }
    
    return AD5231_SendCommand(AD5231_CMD_WRITE_RDAC, position);
}


/**
 * @brief  存储当前RDAC值到EEMEM
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_StoreWiper(void)
{
    return AD5231_SendCommand(AD5231_CMD_STORE_WIPER, 0);
}

/**
 * @brief  从EEMEM(0)恢复RDAC值
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_RestoreEEMEM(void)
{
    return AD5231_SendCommand(AD5231_CMD_RESTORE_EEMEM, 0);
}

/**
 * @brief  复位AD5231 (恢复EEMEM(0)值到RDAC)
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_Reset(void)
{
    return AD5231_SendCommand(AD5231_CMD_RESET, 0);
}

/**
 * @brief  增加1个位置
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_Increment_1(void)
{
    return AD5231_SendCommand(AD5231_CMD_INCREMENT_1, 0);
}

/**
 * @brief  减少1个位置
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_Decrement_1(void)
{
    return AD5231_SendCommand(AD5231_CMD_DECREMENT_1, 0);
}

/**
 * @brief  增加6dB (约增加20%)
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_Increment_6dB(void)
{
    return AD5231_SendCommand(AD5231_CMD_INCREMENT_6DB, 0);
}

/**
 * @brief  减少6dB (约减少20%)
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_Decrement_6dB(void)
{
    return AD5231_SendCommand(AD5231_CMD_DECREMENT_6DB, 0);
}

/**
 * @brief  根据位置值计算电阻
 * @param  position: 位置值 (0-1023)
 * @retval 电阻值(欧姆)
 */
uint32_t AD5231_GetResistance(uint16_t position)
{
    if (position > AD5231_MAX_POSITION) {
        position = AD5231_MAX_POSITION;
    }
    
    // RAB(D) = (D/1024) × RAB + RW
    return (position * AD5231_RESISTANCE_10K / 1024) + AD5231_WIPER_RESISTANCE;
}

/**
 * @brief  设置电阻值
 * @param  resistance_ohm: 目标电阻值(欧姆)
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_SetResistance(uint32_t resistance_ohm)
{
    uint16_t position;
    
    // 检查电阻值范围
    if (resistance_ohm < AD5231_WIPER_RESISTANCE) {
        resistance_ohm = AD5231_WIPER_RESISTANCE;
    }
    if (resistance_ohm > (AD5231_RESISTANCE_10K + AD5231_WIPER_RESISTANCE)) {
        resistance_ohm = AD5231_RESISTANCE_10K + AD5231_WIPER_RESISTANCE;
    }
    
    // 计算位置: D = (RAB - RW) × 1024 / RAB
    position = ((resistance_ohm - AD5231_WIPER_RESISTANCE) * 1024) / AD5231_RESISTANCE_10K;
    
    return AD5231_WriteRDAC(position);
}

/**
 * @brief  设置百分比值
 * @param  percentage: 百分比 (0.0 - 100.0)
 * @retval AD5231_Status_t
 */
AD5231_Status_t AD5231_SetPercentage(float percentage)
{
    uint16_t position;
    
    if (percentage < 0.0f) {
        percentage = 0.0f;
    }
    if (percentage > 100.0f) {
        percentage = 100.0f;
    }
    
    position = (uint16_t)((percentage * AD5231_MAX_POSITION) / 100.0f);
    
    return AD5231_WriteRDAC(position);
}