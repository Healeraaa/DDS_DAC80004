#include "DAC80004.h"
#include "spi.h"



void LDAC(dac80004_bus_t *dev, uint8_t state)
{
    if (state)
    {
        LL_GPIO_SetOutputPin(dev->LDAC_PORT, dev->LDAC_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(dev->LDAC_PORT, dev->LDAC_PIN);
    }
}

void CLR(dac80004_bus_t *dev, uint8_t state)
{
    if (state)
    {
        LL_GPIO_SetOutputPin(dev->CLR_PORT, dev->CLR_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(dev->CLR_PORT, dev->CLR_PIN);
    }
}

void CS(dac80004_bus_t *dev,uint8_t state)
{
    if (state)
    {
        LL_GPIO_SetOutputPin(dev->CS_PORT, dev->CS_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(dev->CS_PORT, dev->CS_PIN);
    }
}
void DAC80004_GPIO_init(dac80004_bus_t *dev)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 根据相应引脚开启时钟
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = dev->LDAC_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;  // 开漏输出
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;                 // 保持无内部上下拉（使用外部上拉）
    LL_GPIO_Init(dev->LDAC_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = dev->CLR_PIN;
    LL_GPIO_Init(dev->CLR_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = dev->CS_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;  //推挽输出
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(dev->CS_PORT, &GPIO_InitStruct);

    // SPI1_Init(); // 初始化SPI

    // CS(dev, 1);  //   取消片选
    // LDAC(dev, 0); // 拉低LDAC
    // CLR(dev, 1); // 拉高CLR 

}

void DAC80004_WR_Config(DAC80004_InitStruct *module, uint8_t WR)
{
    module->WR_Buff = WR;
    // 先清除D31-D28位，再设置新值
    module->TX_Data &= ~(0x0F << 28);        // 清除D31-D28
    module->TX_Data |= ((uint32_t)WR << 28); // 设置新的WR值
}
void DAC80004_Command_Config(DAC80004_InitStruct *module, uint8_t Command)
{
    module->Command_Buff = Command;
    // 先清除D27-D24位，再设置新值
    module->TX_Data &= ~(0x0F << 24);        // 清除D27-D24
    module->TX_Data |= ((uint32_t)Command << 24); // 设置新的Command值
}
void DAC80004_Channel_Config(DAC80004_InitStruct *module, uint8_t Channel)
{
    module->Channel_Buff = Channel;
    // 先清除D23-D20位，再设置新值
    module->TX_Data &= ~(0x0F << 20);        // 清除D23-D20
    module->TX_Data |= ((uint32_t)Channel << 20); // 设置新的Channel值
}
void DAC80004_Data_Set(DAC80004_InitStruct *module, uint16_t Data)
{
    module->Data_Buff = Data;
    // 先清除D19-D4位，再设置新值
    module->TX_Data &= ~(0xFFFF << 4);        // 清除D19-D4
    module->TX_Data |= ((uint32_t)Data << 4); // 设置新的Data值
}
void DAC80004_Mode_Config(DAC80004_InitStruct *module, uint8_t Mode)
{
    module->Mode_Buff = Mode;
    // 先清除D3-D0位，再设置新值
    module->TX_Data &= ~(0x0F);        // 清除D3-D0
    module->TX_Data |= ((uint32_t)Mode); // 设置新的Mode值
}
void DAC80004_WriteData(DAC80004_InitStruct *module)
{
    uint16_t high_word, low_word;
    CS(module->dev, 0); // 使能片选

    // 分解为高16位和低16位
    high_word = (uint16_t)(module->TX_Data >> 16);  // 高16位
    low_word = (uint16_t)(module->TX_Data & 0xFFFF); // 低16位
    
    // 先发送高16位，再发送低16位
    SPI1_Transmit16_Time(high_word,1);
    SPI1_Transmit16_Time(low_word,1);

    CS(module->dev, 1); // 取消片选
}



void DAC80004_Init(DAC80004_InitStruct *module)
{
    DAC80004_GPIO_init(module->dev); // 初始化GPIO
    CS(module->dev, 0);  //   取消片选
    LDAC(module->dev, 0); // 拉低LDAC
    CLR(module->dev, 1); // 拉高CLR 
    SPI1_Init(); // 初始化SPI

    DAC80004_WR_Config(module, DAC_W);
    DAC80004_Command_Config(module, DAC_CMD_WRITE_UPDATE_N);
    DAC80004_Channel_Config(module, DAC_CH_ALL);
    DAC80004_Data_Set(module, 0xFFFF>>1);// 默认中间电压输出
    DAC80004_Mode_Config(module, DAC_MODE_NO);

    DAC80004_WriteData(module);
    
}
void DAC8004_CSL_Config(DAC80004_InitStruct *module, uint8_t state)
{
    if(state)
    {
        CS(module->dev, 1); // 取消片选
    }
    else
    {
        CS(module->dev, 0); // 使能片选
    }
}

