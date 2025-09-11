#ifndef __ECHEM_STIM_H__
#define __ECHEM_STIM_H__

#include "main.h"
#include "stdbool.h"
#include "DAC80004.h"
#include "tim.h"

/**
 * @brief  乒乓DMA状态管理结构体
 */
typedef struct {
    uint8_t active_buffer;          // 当前活跃的缓冲区索引 (0 或 1)
    uint32_t total_points;          // 总数据点数
    uint32_t points_sent;           // 已发送的点数
    uint32_t buffer_size;           // 单个缓冲区大小
    bool transfer_complete;         // 传输完成标志
    bool is_running;                // 运行状态标志
} PingPongDMA_t;

/**
 * @brief  乒乓DMA配置结构体
 */
typedef struct {
    uint32_t buffer_size;           // 缓冲区大小
    double max_sample_rate;         // 最大采样率 (Hz)
    uint32_t min_points;            // 最小点数
    uint32_t max_points;            // 最大点数
    bool enable_progress_callback;  // 启用进度回调
    bool enable_error_recovery;     // 启用错误恢复
} PingPongConfig_t;

/**
 * @brief  电化学刺激方法枚举
 */
typedef enum {
    ECHEM_METHOD_CV = 0,            // 循环伏安法 (Cyclic Voltammetry)
    ECHEM_METHOD_DPV,               // 差分脉冲伏安法 (Differential Pulse Voltammetry)
    ECHEM_METHOD_SWV,               // 方波伏安法 (Square Wave Voltammetry)
    ECHEM_METHOD_CA,                // 计时电流法 (Chronoamperometry)
    ECHEM_METHOD_CC,                // 计时库仑法 (Chronocoulometry)
    ECHEM_METHOD_LSV,               // 线性扫描伏安法 (Linear Sweep Voltammetry)
    ECHEM_METHOD_CUSTOM             // 自定义方法
} EchemMethod_t;

/**
 * @brief  电化学刺激状态枚举
 */
typedef enum {
    ECHEM_STATE_IDLE = 0,           // 空闲状态
    ECHEM_STATE_PREPARING,          // 准备状态
    ECHEM_STATE_RUNNING,            // 运行状态
    ECHEM_STATE_PAUSED,             // 暂停状态
    ECHEM_STATE_COMPLETED,          // 完成状态
    ECHEM_STATE_ERROR               // 错误状态
} EchemState_t;

/**
 * @brief  电化学CV参数结构体
 */
typedef struct {
    double Initial_E;               // 初始电位 (mV)
    double Final_E;                 // 终止电位 (mV)
    double Scan_Limit1;             // 扫描极限1 (mV)
    double Scan_Limit2;             // 扫描极限2 (mV)
    double Scan_Rate;               // 扫描速率 (mV/s)
    uint32_t cycles;                // 循环次数
    double equilibrium_time;        // 平衡时间 (s)
    bool auto_sensitivity;          // 自动灵敏度调节
} EchemCV_Params_t;

/**
 * @brief  电化学DPV参数结构体
 */
typedef struct {
    double Initial_E;               // 初始电位 (mV)
    double Final_E;                 // 终止电位 (mV)
    double Step_E;                  // 步进电位 (mV)
    double Pulse_Amplitude;         // 脉冲幅度 (mV)
    double Pulse_Width;             // 脉冲宽度 (ms)
    double Sample_Width;            // 采样宽度 (ms)
    double Pulse_Period;            // 脉冲周期 (ms)
} EchemDPV_Params_t;

/**
 * @brief  电化学实验结果结构体
 */
typedef struct {
    uint32_t total_points;          // 总数据点数
    double actual_sample_rate;      // 实际采样率 (Hz)
    double total_duration;          // 总持续时间 (s)
    double actual_scan_rate;        // 实际扫描速率 (mV/s)
    double scan_rate_error;         // 扫描速率误差 (mV/s)
    double error_percent;           // 误差百分比 (%)
    
    // 通用状态
    EchemMethod_t method;           // 实验方法
    EchemState_t state;             // 实验状态
    bool success;                   // 实验是否成功
    uint32_t timestamp;             // 时间戳
} EchemResult_t;

/**
 * @brief  进度回调函数类型定义
 * @param  progress: 进度百分比 (0-100)
 * @param  state: 当前实验状态
 * @retval None
 */
typedef void (*EchemProgressCallback_t)(uint8_t progress, EchemState_t state);

/**
 * @brief  错误回调函数类型定义
 * @param  error_state: 错误状态
 * @param  error_code: 错误代码
 * @retval None
 */
typedef void (*EchemErrorCallback_t)(EchemState_t error_state, uint32_t error_code);

// ==================== 常量定义 ====================

// 默认配置常量
#define ECHEM_DEFAULT_BUFFER_SIZE       4096        // 默认缓冲区大小
#define ECHEM_DEFAULT_MAX_SAMPLE_RATE   1000000.0   // 默认最大采样率 1MHz
#define ECHEM_DEFAULT_MIN_POINTS        100         // 默认最小点数
#define ECHEM_DEFAULT_MAX_POINTS        4096        // 默认最大点数

// 电位范围常量
#define ECHEM_VOLTAGE_MIN_MV           -5000.0      // 最小电位 -5V
#define ECHEM_VOLTAGE_MAX_MV           5000.0       // 最大电位 +5V
#define ECHEM_DAC_RESOLUTION           65536        // DAC分辨率

// 扫描速率范围常量
#define ECHEM_SCAN_RATE_MIN_MV_S       0.1          // 最小扫描速率 0.1mV/s
#define ECHEM_SCAN_RATE_MAX_MV_S       10000.0      // 最大扫描速率 10V/s

// 错误代码定义
#define ECHEM_ERROR_NONE               0x00000000   // 无错误
#define ECHEM_ERROR_INVALID_PARAMS     0x00000001   // 参数无效
#define ECHEM_ERROR_DMA_FAILURE        0x00000002   // DMA传输失败
#define ECHEM_ERROR_BUFFER_OVERFLOW    0x00000004   // 缓冲区溢出
#define ECHEM_ERROR_TIMER_CONFIG       0x00000008   // 定时器配置错误
#define ECHEM_ERROR_SPI_BUSY           0x00000010   // SPI总线忙
#define ECHEM_ERROR_MEMORY_ALLOC       0x00000020   // 内存分配失败
#define ECHEM_ERROR_VOLTAGE_RANGE      0x00000040   // 电压超出范围
#define ECHEM_ERROR_SCAN_RATE_RANGE    0x00000080   // 扫描速率超出范围

// 便用宏定义
#define ECHEM_VOLTAGE_TO_DAC(voltage_mv) \
    ((uint16_t)(32768 + ((voltage_mv) * 32768.0 / 5000.0)))

#define ECHEM_DAC_TO_VOLTAGE(dac_value) \
    (((double)(dac_value) - 32768.0) * 5000.0 / 32768.0)

#define ECHEM_IS_VOLTAGE_VALID(voltage_mv) \
    ((voltage_mv) >= ECHEM_VOLTAGE_MIN_MV && (voltage_mv) <= ECHEM_VOLTAGE_MAX_MV)

#define ECHEM_IS_SCAN_RATE_VALID(scan_rate) \
    ((scan_rate) >= ECHEM_SCAN_RATE_MIN_MV_S && (scan_rate) <= ECHEM_SCAN_RATE_MAX_MV_S)


















// ==================== 核心功能函数 ====================

/**
 * @brief  循环伏安法DDS输出 - 乒乓DMA精确版本
 */
bool CV_DDS_Start_Precise(DAC80004_InitStruct *module,
                         const EchemCV_Params_t *cv_params,
                         const PingPongConfig_t *config,
                         uint32_t repeat_count,
                         uint16_t *wave_high_data1, uint16_t *wave_high_data2,
                         uint16_t *wave_low_data1, uint16_t *wave_low_data2);

/**
 * @brief  填充下一个乒乓缓冲区的数据（在主循环中调用）
 */
void CV_Fill_Next_Buffer(DAC80004_InitStruct *module,
                        const EchemCV_Params_t *cv_params,
                        uint32_t initial_points, uint32_t cycle_points, uint32_t final_points);

/**
 * @brief  检查乒乓DMA传输状态
 * @retval true: 传输完成, false: 传输进行中
 */
bool CV_PingPong_IsComplete(void);

/**
 * @brief  获取乒乓DMA传输进度
 * @retval 传输进度百分比 (0-100)
 */
uint8_t CV_PingPong_GetProgress(void);



// ==================== 中断处理函数 ====================

/**
 * @brief  乒乓DMA中断处理函数 - Stream5（高16位）
 */
void CV_PingPong_DMA2_Stream5_IRQHandler(void);

/**
 * @brief  乒乓DMA中断处理函数 - Stream4（低16位）
 */
void CV_PingPong_DMA2_Stream4_IRQHandler(void);













#endif 
