#ifndef __AD5260_H
#define __AD5260_H

#include "main.h"
#include <stdint.h>

// AD5260 参数定义
#define AD5260_MAX_POSITION             255     // 8位分辨率，最大值255
#define AD5260_MIN_POSITION             0       // 最小值0
#define AD5260_RESISTANCE_20K           20000   // 20kΩ标称电阻 (根据型号可选 20k, 50k, 200k)
#define AD5260_WIPER_RESISTANCE         60      // 游标电阻约60Ω

// AD5260 错误代码
typedef enum {
    AD5260_OK = 0,
    AD5260_ERROR_INVALID_POSITION,
    AD5260_ERROR_TIMEOUT,
    AD5260_ERROR_SPI
} AD5260_Status_t;

// 函数声明
void AD5260_Init(void);
void AD5260_CS(uint8_t state);

// 基础操作
AD5260_Status_t AD5260_WriteRDAC(uint8_t position);
AD5260_Status_t AD5260_ReadRDAC(uint8_t *position);

#endif /* __AD5260_H */
