#ifndef __CHANNELGAIN_H
#define __CHANNELGAIN_H

#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

/* WE通道定义 (PB3, PB4控制) */
typedef enum {
    WE_CHANNEL_4 = 0x00,  // PB4=0, PB3=0
    WE_CHANNEL_3 = 0x01,  // PB4=0, PB3=1
    WE_CHANNEL_1 = 0x02,  // PB4=1, PB3=0
    WE_CHANNEL_2 = 0x03   // PB4=1, PB3=1
} WE_Channel_TypeDef;

/* IV转换增益定义 (PB5控制) */
typedef enum {
    IV_GAIN_100   = 0x00,  // PB5=0, 100挡位
    IV_GAIN_10000 = 0x01   // PB5=1, 10000挡位
} IV_Gain_TypeDef;

/* 电压放大增益定义 (PB6, PB7控制) */
typedef enum {
    VOLTAGE_GAIN_1    = 0x00,  // PB7=0, PB6=0, 1倍
    VOLTAGE_GAIN_10   = 0x01,  // PB7=0, PB6=1, 10倍
    VOLTAGE_GAIN_100  = 0x02,  // PB7=1, PB6=0, 100倍
    VOLTAGE_GAIN_1000 = 0x03   // PB7=1, PB6=1, 1000倍
} Voltage_Gain_TypeDef;

/* 反馈通道定义 (PB8, PB9控制) */
typedef enum {
    FB_CHANNEL_4 = 0x00,  // PB9=0, PB8=0
    FB_CHANNEL_3 = 0x01,  // PB9=0, PB8=1
    FB_CHANNEL_1 = 0x02,  // PB9=1, PB8=0
    FB_CHANNEL_2 = 0x03   // PB9=1, PB8=1
} FB_Channel_TypeDef;

/* GPIO端口和引脚定义 */
#define WE_CHANNEL_GPIO_PORT     GPIOB
#define WE_CHANNEL_PIN_A         LL_GPIO_PIN_3   // 低位
#define WE_CHANNEL_PIN_B         LL_GPIO_PIN_4   // 高位

#define IV_GAIN_GPIO_PORT        GPIOB
#define IV_GAIN_PIN              LL_GPIO_PIN_5

#define VOLTAGE_GAIN_GPIO_PORT   GPIOB
#define VOLTAGE_GAIN_PIN_A       LL_GPIO_PIN_6   // 低位
#define VOLTAGE_GAIN_PIN_B       LL_GPIO_PIN_7   // 高位

#define FB_CHANNEL_GPIO_PORT     GPIOB
#define FB_CHANNEL_PIN_A         LL_GPIO_PIN_8   // 低位
#define FB_CHANNEL_PIN_B         LL_GPIO_PIN_9   // 高位

/* 所有控制引脚 */
#define CHANNEL_GAIN_ALL_PINS    (LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
                                  LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9)

/* 函数声明 */
void ChannelGain_Init(void);
void ChannelGain_GPIO_Init(void);

void WE_Channel_Select(WE_Channel_TypeDef channel);
WE_Channel_TypeDef WE_Channel_GetCurrent(void);

void IV_Gain_Set(IV_Gain_TypeDef gain);
IV_Gain_TypeDef IV_Gain_GetCurrent(void);

void Voltage_Gain_Set(Voltage_Gain_TypeDef gain);
Voltage_Gain_TypeDef Voltage_Gain_GetCurrent(void);

void FB_Channel_Select(FB_Channel_TypeDef channel);
FB_Channel_TypeDef FB_Channel_GetCurrent(void);

#endif /* __CHANNELGAIN_H */