#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>
#include "usart.h"

// 协议定义
#define SERIAL_PACKET_HEADER    0xFF
#define SERIAL_PACKET_TAIL      0xFE
#define SERIAL_DATA_LENGTH      5
#define SERIAL_TIMEOUT_MS       100

// 接收状态枚举
typedef enum {
    SERIAL_STATE_WAIT_HEADER = 0,
    SERIAL_STATE_RECEIVE_DATA,
    SERIAL_STATE_WAIT_TAIL
} SerialRxState_t;

typedef union {
    double      double_val;                 // IEEE 754 双精度浮点数 (64位)
    uint8_t     u8_array[sizeof(double)];   // 字节数组表示
} Serial_DoubleConverter_t;

// 双缓冲区结构体
typedef struct {
    uint8_t write_buffer[SERIAL_DATA_LENGTH];    // 中断写入缓冲区
    uint8_t read_buffer[SERIAL_DATA_LENGTH];     // 用户读取缓冲区
    uint8_t length;                              // 当前接收长度
    uint8_t is_ready;                            // 数据包完整标志
    uint8_t buffer_swapped;                      // 缓冲区交换标志
    SerialRxState_t state;                       // 接收状态
    uint32_t timeout_counter;                    // 超时计数器
} SerialPacket_t;

union SerialDoubeleBuffer_t {
    double doubele_data;
    uint8_t data[8];
};





uint8_t Serial_TransmitByte(USART_TypeDef *USARTx, uint8_t data, uint32_t timeout_ms);
uint8_t Serial_TransmitData(USART_TypeDef *USARTx, uint8_t *data, uint8_t length, uint32_t timeout_ms);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(uint8_t *data, uint8_t length);
void USART1_IRQ_Task(void);
void Serial_TimeoutCheck(void);
void USART_CommandTask(uint8_t command);

#endif
