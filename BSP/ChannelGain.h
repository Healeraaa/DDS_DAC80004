#ifndef __CHANNELGAIN_H
#define __CHANNELGAIN_H

#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

/* WE通道定义 (PB3, PB4控制四选一模拟开关) */
typedef enum {
    WE_CHANNEL_4 = 0x00,  // PB4=0, PB3=0
    WE_CHANNEL_3 = 0x01,  // PB4=0, PB3=1
    WE_CHANNEL_1 = 0x02,  // PB4=1, PB3=0
    WE_CHANNEL_2 = 0x03   // PB4=1, PB3=1
} WE_Channel_TypeDef;

/* IV转换倍数定义 (PB5, PB6控制四选一模拟开关) */
typedef enum {
    IV_GAIN_1K   = 0x00,  // PB6=0, PB5=0, 1K
    IV_GAIN_100K = 0x01,  // PB6=0, PB5=1, 100K
    IV_GAIN_10M  = 0x02,  // PB6=1, PB5=0, 10M
    IV_GAIN_100M = 0x03   // PB6=1, PB5=1, 100M
} IV_Gain_TypeDef;

/* 第一级电压放大倍数定义 (PB7控制二选一模拟开关) */
typedef enum {
    VOLTAGE_GAIN_STAGE1_5X  = 0x00,  // PB7=0, 5倍
    VOLTAGE_GAIN_STAGE1_20X = 0x01   // PB7=1, 20倍
} Voltage_Gain_Stage1_TypeDef;

/* 第二级电压放大倍数定义 (PB8, PB9控制四选一模拟开关) */
typedef enum {
    VOLTAGE_GAIN_STAGE2_1X  = 0x00,  // PB9=0, PB8=0, 1倍
    VOLTAGE_GAIN_STAGE2_5X  = 0x01,  // PB9=0, PB8=1, 5倍
    VOLTAGE_GAIN_STAGE2_20X = 0x02,  // PB9=1, PB8=0, 20倍
    VOLTAGE_GAIN_STAGE2_50X = 0x03   // PB9=1, PB8=1, 50倍
} Voltage_Gain_Stage2_TypeDef;

/* 反馈选择定义 (二选一模拟开关) */
typedef enum {
    FEEDBACK_GND = 0x00,  // 选择GND
    FEEDBACK_FB  = 0x01   // 选择FB
} Feedback_Select_TypeDef;

/* GPIO端口和引脚定义 */

/* WE通道选择 (PB3, PB4) */
#define WE_CHANNEL_GPIO_PORT     GPIOB
#define WE_CHANNEL_PIN_A         LL_GPIO_PIN_3   // 低位
#define WE_CHANNEL_PIN_B         LL_GPIO_PIN_4   // 高位

/* IV转换倍数 (PB5, PB6) */
#define IV_GAIN_GPIO_PORT        GPIOB
#define IV_GAIN_PIN_A            LL_GPIO_PIN_5   // 低位
#define IV_GAIN_PIN_B            LL_GPIO_PIN_6   // 高位

/* 第一级电压放大 (PB7) */
#define VOLTAGE_GAIN_STAGE1_GPIO_PORT   GPIOB
#define VOLTAGE_GAIN_STAGE1_PIN         LL_GPIO_PIN_7

/* 第二级电压放大 (PB8, PB9) */
#define VOLTAGE_GAIN_STAGE2_GPIO_PORT   GPIOB
#define VOLTAGE_GAIN_STAGE2_PIN_A       LL_GPIO_PIN_8   // 低位
#define VOLTAGE_GAIN_STAGE2_PIN_B       LL_GPIO_PIN_9   // 高位

/* 反馈选择1 (PC10) */
#define FEEDBACK1_GPIO_PORT      GPIOC
#define FEEDBACK1_PIN            LL_GPIO_PIN_10

/* 反馈选择2 (PC11) */
#define FEEDBACK2_GPIO_PORT      GPIOC
#define FEEDBACK2_PIN            LL_GPIO_PIN_11

/* 反馈选择3 (PC12) */
#define FEEDBACK3_GPIO_PORT      GPIOC
#define FEEDBACK3_PIN            LL_GPIO_PIN_12

/* 反馈选择4 (PD2) */
#define FEEDBACK4_GPIO_PORT      GPIOD
#define FEEDBACK4_PIN            LL_GPIO_PIN_2

/* GPIOB所有控制引脚 */
#define CHANNEL_GAIN_GPIOB_PINS  (LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
                                  LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9)

/* GPIOC所有控制引脚 */
#define CHANNEL_GAIN_GPIOC_PINS  (LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12)

/* GPIOD所有控制引脚 */
#define CHANNEL_GAIN_GPIOD_PINS  (LL_GPIO_PIN_2)

/* 函数声明 */
void ChannelGain_Init(void);
void ChannelGain_GPIO_Init(void);

/* WE通道选择函数 */
void WE_Channel_Select(WE_Channel_TypeDef channel);
WE_Channel_TypeDef WE_Channel_GetCurrent(void);

/* IV转换倍数设置函数 */
void IV_Gain_Set(IV_Gain_TypeDef gain);
IV_Gain_TypeDef IV_Gain_GetCurrent(void);

/* 第一级电压放大设置函数 */
void Voltage_Gain_Stage1_Set(Voltage_Gain_Stage1_TypeDef gain);
Voltage_Gain_Stage1_TypeDef Voltage_Gain_Stage1_GetCurrent(void);

/* 第二级电压放大设置函数 */
void Voltage_Gain_Stage2_Set(Voltage_Gain_Stage2_TypeDef gain);
Voltage_Gain_Stage2_TypeDef Voltage_Gain_Stage2_GetCurrent(void);

/* 反馈选择函数 */
void Feedback1_Select(Feedback_Select_TypeDef select);
Feedback_Select_TypeDef Feedback1_GetCurrent(void);

void Feedback2_Select(Feedback_Select_TypeDef select);
Feedback_Select_TypeDef Feedback2_GetCurrent(void);

void Feedback3_Select(Feedback_Select_TypeDef select);
Feedback_Select_TypeDef Feedback3_GetCurrent(void);

void Feedback4_Select(Feedback_Select_TypeDef select);
Feedback_Select_TypeDef Feedback4_GetCurrent(void);

#endif /* __CHANNELGAIN_H */
