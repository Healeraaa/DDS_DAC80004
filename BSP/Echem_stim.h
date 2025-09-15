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
    volatile bool need_fill_buffer; // 需要填充缓冲区标志
    volatile uint8_t buffer_to_fill;// 需要填充的缓冲区索引 (0 或 1)
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
    
    // 计算出的段点数（由系统自动计算填充）
    uint32_t initial_points;        // 起始段点数
    uint32_t cycle_points;          // 循环段点数  
    uint32_t final_points;          // 结束段点数
    uint32_t total_points;          // 总点数
    double actual_sample_rate;      // 实际采样率 (Hz)
} EchemCV_Params_t;
/**
 * @brief  电化学DPV参数结构体（
 */
typedef struct {
    double Initial_E;               // 初始电位 (mV)
    double Final_E;                 // 终止电位 (mV)
    double Step_E;                  // 步进电位 (mV)
    double Pulse_Amplitude;         // 脉冲幅度 (mV)
    double Pulse_Width;             // 脉冲宽度 (ms)
    double Pulse_Period;            // 脉冲周期 (ms)
    double equilibrium_time;        // 平衡时间 (s)
    bool auto_sensitivity;          // 自动灵敏度调节
    
    // 计算出的参数（由系统自动计算填充）
    uint32_t total_steps;           // 总步数
    uint32_t points_per_step;       // 每步的点数
    uint32_t pulse_points;          // 脉冲部分点数
    uint32_t base_points;           // 基础部分点数
    uint32_t total_points;          // 总点数
    double actual_sample_rate;      // 实际采样率 (Hz)
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
#define ECHEM_DEFAULT_BUFFER_SIZE       1024*8        // 默认缓冲区大小
#define ECHEM_DEFAULT_MAX_SAMPLE_RATE   85000.0   // 默认最大采样率 1MHz
#define ECHEM_DEFAULT_MIN_POINTS        100         // 默认最小点数
#define ECHEM_DEFAULT_MAX_POINTS        1024*8         // 默认最大点数

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
#define ECHEM_VOLTAGE_RATE_TO_DAC_RATE(voltage_mv) \
    ((uint16_t)(32768 + ((voltage_mv) * 32768.0 / 5000.0)))

#define ECHEM_VOLTAGE_TO_DAC(voltage_mv) \
    ((uint16_t)((voltage_mv) * (0xFFFF) / 10000.0))

#define ECHEM_DAC_TO_VOLTAGE(dac_value) \
    (((double)(dac_value) - 32768.0) * 5000.0 / 32768.0)

#define ECHEM_IS_VOLTAGE_VALID(voltage_mv) \
    ((voltage_mv) >= ECHEM_VOLTAGE_MIN_MV && (voltage_mv) <= ECHEM_VOLTAGE_MAX_MV)

#define ECHEM_IS_SCAN_RATE_VALID(scan_rate) \
    ((scan_rate) >= ECHEM_SCAN_RATE_MIN_MV_S && (scan_rate) <= ECHEM_SCAN_RATE_MAX_MV_S)

// ==================== DPV常量定义 ====================
#define DPV_PULSE_WIDTH_MIN_MS         0.1          // 最小脉冲宽度 0.1ms
#define DPV_PULSE_WIDTH_MAX_MS         1000.0       // 最大脉冲宽度 1s
#define DPV_PULSE_PERIOD_MIN_MS        1.0          // 最小脉冲周期 1ms
#define DPV_PULSE_PERIOD_MAX_MS        10000.0      // 最大脉冲周期 10s
#define DPV_STEP_E_MIN_MV              0.1          // 最小步进电位 0.1mV
#define DPV_STEP_E_MAX_MV              1000.0       // 最大步进电位 1V
#define DPV_PULSE_AMPLITUDE_MIN_MV     1.0          // 最小脉冲幅度 1mV
#define DPV_PULSE_AMPLITUDE_MAX_MV     500.0        // 最大脉冲幅度 500mV

// DPV参数验证宏
#define DPV_IS_PULSE_WIDTH_VALID(width_ms) \
    ((width_ms) >= DPV_PULSE_WIDTH_MIN_MS && (width_ms) <= DPV_PULSE_WIDTH_MAX_MS)

#define DPV_IS_PULSE_PERIOD_VALID(period_ms) \
    ((period_ms) >= DPV_PULSE_PERIOD_MIN_MS && (period_ms) <= DPV_PULSE_PERIOD_MAX_MS)

#define DPV_IS_STEP_E_VALID(step_e_mv) \
    ((step_e_mv) >= DPV_STEP_E_MIN_MV && (step_e_mv) <= DPV_STEP_E_MAX_MV)

#define DPV_IS_PULSE_AMPLITUDE_VALID(amplitude_mv) \
    ((amplitude_mv) >= DPV_PULSE_AMPLITUDE_MIN_MV && (amplitude_mv) <= DPV_PULSE_AMPLITUDE_MAX_MV)





// ==================== 通用乒乓DMA管理函数 ====================

/**
 * @brief  停止乒乓DMA传输
 * @retval None
 */
void PingPong_DMA_Stop(void);

/**
 * @brief  暂停乒乓DMA传输
 * @retval None
 */
void PingPong_DMA_Pause(void);

/**
 * @brief  恢复乒乓DMA传输
 * @retval None
 */
void PingPong_DMA_Resume(void);

/**
 * @brief  检查乒乓DMA传输状态
 * @retval true: 传输完成, false: 传输进行中
 */
bool PingPong_DMA_IsComplete(void);

/**
 * @brief  获取乒乓DMA传输进度
 * @retval 传输进度百分比 (0-100)
 */
uint8_t PingPong_DMA_GetProgress(void);

/**
 * @brief  重置乒乓DMA状态
 * @retval None
 */
void PingPong_DMA_Reset(void);

// ==================== CV专用函数 ====================

/**
 * @brief  循环伏安法DDS输出 - 乒乓DMA精确版本
 */
bool CV_DDS_Start_Precise(DAC80004_InitStruct *module,
                         const EchemCV_Params_t *cv_params,
                         const PingPongConfig_t *config,
                         uint16_t *wave_high_data1, uint16_t *wave_high_data2,
                         uint16_t *wave_low_data1, uint16_t *wave_low_data2);

/**
 * @brief  填充下一个CV缓冲区的数据（在主循环中调用）
 */
void CV_Fill_Next_Buffer(void);

/**
 * @brief  检查是否需要填充CV缓冲区
 * @retval true: 需要填充, false: 不需要填充
 */
bool CV_NeedFillBuffer(void);

// ==================== DPV专用函数 ====================
/**
 * @brief  差分脉冲伏安法DDS输出 - 乒乓DMA精确版本
 */
bool DPV_DDS_Start_Precise(DAC80004_InitStruct *module,
                          const EchemDPV_Params_t *dpv_params,
                          const PingPongConfig_t *config,
                          uint16_t *wave_high_data1, uint16_t *wave_high_data2,
                          uint16_t *wave_low_data1, uint16_t *wave_low_data2);

/**
 * @brief  填充下一个DPV缓冲区的数据
 */
void DPV_Fill_Next_Buffer(void);

/**
 * @brief  检查是否需要填充DPV缓冲区
 */
bool DPV_NeedFillBuffer(void);



// ==================== 实验结果和回调管理 ====================

void Echem_stim_Init(DAC80004_InitStruct *module);

/**
 * @brief  获取实验结果
 */
const EchemResult_t* Echem_GetResult(void);

/**
 * @brief  重置实验状态
 */
void Echem_Reset(void);

/**
 * @brief  设置进度回调函数
 */
void Echem_SetProgressCallback(EchemProgressCallback_t callback);

/**
 * @brief  设置错误回调函数
 */
void Echem_SetErrorCallback(EchemErrorCallback_t callback);







#endif 
