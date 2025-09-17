#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>
#include "usart.h"

// 协议定义
#define SERIAL_PACKET_HEADER    0xFF
#define SERIAL_PACKET_TAIL      0xFE
#define SERIAL_DATA_LENGTH      10 // 数据长度（double）
#define SERIAL_TIMEOUT_MS       100

// 接收状态枚举
// 在Serial.h中添加
typedef enum {
    SERIAL_STATE_WAIT_HEADER = 0,        // 等待包头
    SERIAL_STATE_RECEIVE_COMMAND,        // 接收命令字
    SERIAL_STATE_RECEIVE_DATA,           // 接收数据
    SERIAL_STATE_WAIT_TAIL              // 等待包尾
} Serial_RxState_t;

typedef union {
    double      double_val;                 // IEEE 754 双精度浮点数 (64位)
    uint8_t     u8_array[sizeof(double)];   // 字节数组表示
} Serial_DoubleConverter_t;

// 双缓冲区结构体

typedef struct {
    uint8_t command_write_buffer;                  // 命令字写缓存区
    uint8_t command_read_buffer;                  // 命令字读缓存区
    Serial_DoubleConverter_t write_buffer[SERIAL_DATA_LENGTH];        // 写缓冲区
    Serial_DoubleConverter_t read_buffer[SERIAL_DATA_LENGTH];         // 读缓冲区
    uint8_t expected_length;        // 期望接收的数据长度
    uint8_t current_length;         // 当前接收长度
    uint8_t is_ready;               // 数据包完整标志
    Serial_RxState_t state;          // 接收状态
    uint32_t timeout_counter;       // 超时计数器
} Serial_Packet_t;



uint8_t Serial_TransmitByte(USART_TypeDef *USARTx, uint8_t data, uint32_t timeout_ms);
uint8_t Serial_GetRxCommand(void);
uint8_t Serial_TransmitData(USART_TypeDef *USARTx, uint8_t *data, uint8_t length, uint32_t timeout_ms);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(double *data, uint8_t length);
void USART1_IRQ_Task(void);
void Serial_TimeoutCheck(void);

#endif
