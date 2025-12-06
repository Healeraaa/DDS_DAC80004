#ifndef __AD5231_H
#define __AD5231_H

#include "main.h"
#include <stdint.h>

// AD5231 命令定义
#define AD5231_CMD_NOP                  0x00    // 无操作
#define AD5231_CMD_RESTORE_EEMEM        0x01    // 从EEMEM恢复RDAC值
#define AD5231_CMD_STORE_WIPER          0x02    // 存储游标设置到EEMEM
#define AD5231_CMD_STORE_TO_EEMEM       0x03    // 存储数据到EEMEM (16位)
#define AD5231_CMD_DECREMENT_6DB        0x04    // 减少6dB
#define AD5231_CMD_DECREMENT_6DB_ALL    0x05    // 所有通道减少6dB
#define AD5231_CMD_DECREMENT_1          0x06    // 减少1个位置
#define AD5231_CMD_DECREMENT_1_ALL      0x07    // 所有通道减少1个位置
#define AD5231_CMD_RESET                0x08    // 复位，恢复EEMEM(0)值
#define AD5231_CMD_READ_EEMEM           0x09    // 从SDO读取EEMEM
#define AD5231_CMD_READ_RDAC            0x0A    // 从SDO读取RDAC游标设置
#define AD5231_CMD_WRITE_RDAC           0x0B    // 写入RDAC (10位)
#define AD5231_CMD_INCREMENT_6DB        0x0C    // 增加6dB
#define AD5231_CMD_INCREMENT_6DB_ALL    0x0D    // 所有通道增加6dB
#define AD5231_CMD_INCREMENT_1          0x0E    // 增加1个位置
#define AD5231_CMD_INCREMENT_1_ALL      0x0F    // 所有通道增加1个位置

// AD5231 参数定义
#define AD5231_MAX_POSITION             1023    // 10位分辨率，最大值1023
#define AD5231_MIN_POSITION             0       // 最小值0
#define AD5231_RESISTANCE_10K           10000   // 10kΩ标称电阻
#define AD5231_WIPER_RESISTANCE         60      // 游标电阻约60Ω

// AD5231 错误代码
typedef enum {
    AD5231_OK = 0,
    AD5231_ERROR_INVALID_POSITION,
    AD5231_ERROR_TIMEOUT,
    AD5231_ERROR_SPI
} AD5231_Status_t;

// 函数声明
void AD5231_Init(void);
void AD5231_CS(uint8_t state);

// 基础操作
AD5231_Status_t AD5231_WriteRDAC(uint16_t position);
AD5231_Status_t AD5231_StoreWiper(void);
AD5231_Status_t AD5231_RestoreEEMEM(void);
AD5231_Status_t AD5231_Reset(void);

// 增量/减量操作
AD5231_Status_t AD5231_Increment_1(void);
AD5231_Status_t AD5231_Decrement_1(void);
AD5231_Status_t AD5231_Increment_6dB(void);
AD5231_Status_t AD5231_Decrement_6dB(void);

// 高级操作
AD5231_Status_t AD5231_SetResistance(uint32_t resistance_ohm);
AD5231_Status_t AD5231_SetPercentage(float percentage);
uint32_t AD5231_GetResistance(uint16_t position);

#endif /* __AD5231_H */