#include "Serial.h"

static SerialPacket_t g_serial_rx_packet = {
    .state = SERIAL_STATE_WAIT_HEADER,
    .expected_length = SERIAL_DATA_LENGTH * sizeof(double), // 期望接收SERIAL_DATA_LENGTH个double数据
    .current_length = 0,
    .is_ready = 0,
    .timeout_counter = 0
};

/**
 * @brief  安全发送单个字节数据
 * @param  USARTx: USART实例
 * @param  data: 要发送的数据
 * @param  timeout_ms: 超时时间（毫秒），0表示无超时
 * @retval 0: 发送成功, 1: 超时失败
 */
uint8_t Serial_TransmitByte(USART_TypeDef *USARTx, uint8_t data, uint32_t timeout_ms)
{
    uint32_t timeout_counter = 0;
    
    // 等待发送数据寄存器为空
    while (!LL_USART_IsActiveFlag_TXE(USARTx)) {
        if (timeout_ms > 0) {
            timeout_counter++;
            if (timeout_counter >= timeout_ms * 1000) {  // 假设1ms = 1000次循环
                return 1;  // 超时
            }
        }
    }
    // 发送数据
    LL_USART_TransmitData8(USARTx, data);
    
    return 0;  // 成功
}

/**
 * @brief  安全发送数据数组
 * @param  USARTx: USART实例
 * @param  data: 要发送的数据数组
 * @param  length: 数据长度
 * @param  timeout_ms: 单个字节超时时间（毫秒），0表示无超时
 * @retval 发送成功的字节数
 */
uint8_t Serial_TransmitData(USART_TypeDef *USARTx, uint8_t *data, uint8_t length, uint32_t timeout_ms)
{
    if (data == NULL || length == 0) {
        return 0;
    }
    
    uint8_t sent_count = 0;
    
    for (uint8_t i = 0; i < length; i++) {
        if (Serial_TransmitByte(USARTx, data[i], timeout_ms) == 0) {
            sent_count++;
        } else {
            break;  // 发送失败，停止发送
        }
    }
    
    // 等待最后一个字节发送完成
    uint32_t timeout_counter = 0;
    while (!LL_USART_IsActiveFlag_TC(USARTx)) {
        if (timeout_ms > 0) {
            timeout_counter++;
            if (timeout_counter >= timeout_ms * 1000) {
                break;  // 超时
            }
        }
    }
    
    return sent_count;
}



/**
 * @brief  获取串口接收数据包标志位
 * @retval 1: 有新数据包, 0: 无数据包
 */
uint8_t Serial_GetRxFlag(void)
{
    if (g_serial_rx_packet.is_ready) {
        g_serial_rx_packet.is_ready = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief  获取接收到的数据包
 * @param  data: 输出缓冲区
 * @param  length: 缓冲区长度
 * @retval 实际复制的数据长度
 */
uint8_t Serial_GetRxData(double *data, uint8_t length)
{
    if (data == NULL || length == 0) {
        return 0;
    }
    
    uint8_t copy_length = (length > SERIAL_DATA_LENGTH) ? SERIAL_DATA_LENGTH : length;
    
    // 从读缓冲区复制数据，保证数据完整性
    for (uint8_t i = 0; i < copy_length; i++) {
        data[i] = g_serial_rx_packet.read_buffer[i].double_val;
    }
    
    return copy_length;
}

/**
 * @brief  重置接收状态机（保留数据标志）
 */
static void Serial_ResetRxStateOnly(void)
{
    g_serial_rx_packet.state = SERIAL_STATE_WAIT_HEADER;
    g_serial_rx_packet.current_length = 0;
    g_serial_rx_packet.timeout_counter = 0;
    // 不重置 is_ready，让主循环处理
}

/**
 * @brief  重置接收状态机
 */
static void Serial_ResetRxState(void)
{
    g_serial_rx_packet.state = SERIAL_STATE_WAIT_HEADER;
    g_serial_rx_packet.current_length = 0;
    g_serial_rx_packet.is_ready = 0;
    g_serial_rx_packet.timeout_counter = 0;
}

/**
 * @brief  数据包接收完成处理函数
 */
static void Serial_PacketComplete(void)
{
    // 将写缓冲区数据复制到读缓冲区
    for (uint8_t i = 0; i < SERIAL_DATA_LENGTH; i++) {
        g_serial_rx_packet.read_buffer[i].double_val = g_serial_rx_packet.write_buffer[i].double_val;
    }
    
    g_serial_rx_packet.is_ready = 1;  // 设置数据包完整标志
    Serial_ResetRxStateOnly();        // 重置状态，保留数据标志
}

/**
 * @brief  串口接收超时检查（需要在定时器中调用）
 */
void Serial_TimeoutCheck(void)
{
    if (g_serial_rx_packet.state != SERIAL_STATE_WAIT_HEADER) {
        g_serial_rx_packet.timeout_counter++;
        
        if (g_serial_rx_packet.timeout_counter > SERIAL_TIMEOUT_MS) {
            // 超时，重置状态机
            Serial_ResetRxState();
        }
    }
}

/**
 * @brief  串口接收中断处理函数
 */
void USART1_IRQ_Task(void)
{
    if (LL_USART_IsActiveFlag_RXNE(USART1) == SET) {
        uint8_t rx_data = LL_USART_ReceiveData8(USART1);
        
        switch (g_serial_rx_packet.state) {
            case SERIAL_STATE_WAIT_HEADER:
                if (rx_data == SERIAL_PACKET_HEADER) {
                    g_serial_rx_packet.state = SERIAL_STATE_RECEIVE_DATA;
                    g_serial_rx_packet.current_length = 0;
                    g_serial_rx_packet.timeout_counter = 0;
                }
                break;
                
            case SERIAL_STATE_RECEIVE_DATA:
                if (g_serial_rx_packet.current_length < g_serial_rx_packet.expected_length) {
                    uint8_t double_index = g_serial_rx_packet.current_length / sizeof(double);
                    uint8_t byte_index = g_serial_rx_packet.current_length % sizeof(double);
                    
                    // 确保不会越界
                    if (double_index < SERIAL_DATA_LENGTH) {
                        g_serial_rx_packet.write_buffer[double_index].u8_array[byte_index] = rx_data;
                        g_serial_rx_packet.current_length++;
                        
                        if (g_serial_rx_packet.current_length >= g_serial_rx_packet.expected_length) {
                            g_serial_rx_packet.state = SERIAL_STATE_WAIT_TAIL;
                        }
                    } else {
                        // 越界保护
                        Serial_ResetRxState();
                    }
                } else {
                    // 数据溢出，重置状态机
                    Serial_ResetRxState();
                }
                break;
                
            case SERIAL_STATE_WAIT_TAIL:
                if (rx_data == SERIAL_PACKET_TAIL) {
                    // 据包接收完成，调用处理函数
                    Serial_PacketComplete();
                } 
                else {
                    // 包尾错误，重置状态机
                    Serial_ResetRxState();
                }
                break;
                
            default:
                Serial_ResetRxState();
                break;
        }
        
        LL_USART_ClearFlag_RXNE(USART1);
    }
}





/**
 * @brief  串口命令处理函数
 * @param  command: 命令字节
 */
void USART_CommandTask(uint8_t command)
{

}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    USART1_IRQ_Task();
}