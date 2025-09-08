
#ifndef __TIM_H__
#define __TIM_H__


#include "main.h"

typedef struct {
    uint32_t prescaler;      // 预分频器值 (寄存器值，实际分频 = prescaler + 1)
    uint32_t period;         // 自动重装载值 (寄存器值，实际周期 = period + 1)
    double actual_freq;    // 实际输出频率
    double error;          // 频率误差
} TIM_FreqConfig_t;

void TIM1_DMA_SPI1_Init(void);
uint8_t TIM_CalculateFreqDivision_Precise(uint32_t timer_clock, double target_freq, TIM_FreqConfig_t *config);
void TIM_ApplyFreqConfig(TIM_TypeDef *TIMx, const TIM_FreqConfig_t *config);
void TIM_ApplyFreqConfig_DualDMA(TIM_TypeDef *TIMx, const TIM_FreqConfig_t *config, double timer_clock, double spi_baudrate);
void TIM3_PWM_Init(void);
void TIM3_ApplyPWMConfig(const TIM_FreqConfig_t *config, double timer_clock, double spi_baudrate);
void TIM3_PWM_Start(void);
void TIM3_PWM_Stop(void);


#endif /* __TIM_H__ */

