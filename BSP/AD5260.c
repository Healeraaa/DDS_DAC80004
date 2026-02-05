#include "AD5260.h"
#include "spi.h"

/**
 * @brief  AD5231 片选控制
 * @param  state: 1=取消片选(CS高), 0=选中(CS低)
 */
void AD5260_CS(uint8_t state)
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
void AD5260_Init(void)
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
    AD5260_CS(1);
    
    // 延时等待芯片稳定
    LL_mDelay(10);
}

/**
 * @brief  写入RDAC寄存器
 * @param  position: 游标位置 (0-255)
 * @retval AD5260_Status_t
 */
AD5260_Status_t AD5260_WriteRDAC(uint8_t position)
{
    HAL_StatusTypeDef status;
    
    AD5260_CS(0);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    SPI_Transmit8_Time(SPI2, position, 1);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    AD5260_CS(1);
    
    if (status != HAL_OK)
    {
        return AD5260_ERROR_SPI;
    }
    
    return AD5260_OK;
}