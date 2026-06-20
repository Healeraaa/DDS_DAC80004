/**
 * @file GPCI_Example.c
 * @brief GPCI方法使用示例
 * 
 * 演示如何使用新实现的GPCI（门控脉冲激发与循环积分法）
 */

#include "Echem_stim.h"
#include "DAC80004.h"
#include "tim.h"
#include "dma.h"

// 缓冲区定义（与CV/DPV/CA相同的方式）
#define BUFFER_SIZE (1024 * 8)  // 8KB缓冲区

// 乒乓DMA缓冲区
uint16_t gpci_wave_high_data1[BUFFER_SIZE] = {0};
uint16_t gpci_wave_high_data2[BUFFER_SIZE] = {0};
uint16_t gpci_wave_low_data1[BUFFER_SIZE] = {0};
uint16_t gpci_wave_low_data2[BUFFER_SIZE] = {0};

/**
 * @brief GPCI实验示例函数
 * @details 
 *   - 预激发电位: -200 mV
 *   - 反应电位: 500 mV (脉冲激发电位)
 *   - 恢复电位: -500 mV
 *   - 静息恢复总时间: 2.0 s
 *   - 脉冲激发时长: 0.5 s
 *   - 循环次数: 10
 * 
 *   运行时序：
 *   每个循环的三个阶段:
 *   1. 输出 -200mV，持续 1.0s (2.0s / 2)
 *   2. 输出 +500mV，持续 0.5s
 *   3. 输出 -500mV，持续 1.0s (2.0s / 2)
 *   
 *   共循环10次
 */
void GPCI_Electrochemistry_Example(DAC80004_InitStruct *dac_module)
{
    // 1. 定义GPCI参数
    EchemGPCI_Params_t gpci_params = {
        .Pre_Excitation_E = -200.0,          // 预激发电位 (-200 mV)
        .Response_E = 500.0,                 // 反应电位 (+500 mV，脉冲激发)
        .Recovery_E = -500.0,                // 恢复电位 (-500 mV)
        .Rest_Recovery_Total_Time = 2.0,     // 静息恢复总时间 (2.0 s)
        .Pulse_Duration = 0.5,               // 脉冲激发时长 (0.5 s)
        .Cycles = 10,                        // 循环总次数 (10次)
        .auto_sensitivity = false            // 不使用自动灵敏度调节
    };
    
    // 2. 定义乒乓DMA配置
    PingPongConfig_t ping_pong_config = {
        .buffer_size = BUFFER_SIZE,
        .max_sample_rate = 85000.0,          // 最大采样率 85kHz
        .min_points = 100,
        .max_points = BUFFER_SIZE,
        .enable_progress_callback = true,
        .enable_error_recovery = true
    };
    
    // 3. 启动GPCI实验
    bool start_success = GPCI_DDS_Start_Precise(
        dac_module,
        &gpci_params,
        &ping_pong_config,
        gpci_wave_high_data1, gpci_wave_high_data2,
        gpci_wave_low_data1, gpci_wave_low_data2
    );
    
    if (!start_success) {
        // 处理启动失败的情况
        return;
    }
    
    // 4. 在主循环中调用GPCI_Fill_Next_Buffer()来填充缓冲区
    // 这应该在你的主程序循环中被调用，例如：
    // while (1) {
    //     if (GPCI_NeedFillBuffer()) {
    //         GPCI_Fill_Next_Buffer();
    //     }
    //     // 其他任务...
    // }
    
    // 5. 等待实验完成
    const EchemResult_t *result = Echem_GetResult();
    while (result->state != ECHEM_STATE_COMPLETED && 
           result->state != ECHEM_STATE_ERROR) {
        // 可以在这里处理其他事务
        if (GPCI_NeedFillBuffer()) {
            GPCI_Fill_Next_Buffer();
        }
    }
    
    // 6. 检查实验结果
    if (result->success && result->state == ECHEM_STATE_COMPLETED) {
        // 实验成功完成
        // 使用result中的信息：
        // - total_points: 总数据点数
        // - actual_sample_rate: 实际采样率
        // - total_duration: 总持续时间
        // - method: ECHEM_METHOD_GPCI
    } else {
        // 实验出错或失败
    }
}

/**
 * @brief GPCI实验2 - 更快速的脉冲
 * @details
 *   - 预激发电位: 0 mV
 *   - 反应电位: 800 mV
 *   - 恢复电位: -200 mV
 *   - 静息恢复总时间: 0.2 s
 *   - 脉冲激发时长: 0.05 s (50ms)
 *   - 循环次数: 20
 * 
 *   这是一个更快速的GPCI扫描示例
 */
void GPCI_FastPulse_Example(DAC80004_InitStruct *dac_module)
{
    EchemGPCI_Params_t gpci_params = {
        .Pre_Excitation_E = 0.0,             // 0 mV
        .Response_E = 800.0,                 // +800 mV
        .Recovery_E = -200.0,                // -200 mV
        .Rest_Recovery_Total_Time = 0.2,     // 0.2 s
        .Pulse_Duration = 0.05,              // 0.05 s (50ms)
        .Cycles = 20,                        // 20次循环
        .auto_sensitivity = false
    };
    
    PingPongConfig_t config = {
        .buffer_size = BUFFER_SIZE,
        .max_sample_rate = 85000.0,
        .min_points = 100,
        .max_points = BUFFER_SIZE,
        .enable_progress_callback = true,
        .enable_error_recovery = true
    };
    
    GPCI_DDS_Start_Precise(
        dac_module,
        &gpci_params,
        &config,
        gpci_wave_high_data1, gpci_wave_high_data2,
        gpci_wave_low_data1, gpci_wave_low_data2
    );
}

/**
 * @brief 在主循环中的集成
 * 
 * 示例代码：
 * 
 * int main(void) {
 *     // ... 初始化代码 ...
 *     DAC80004_InitStruct dac_module;
 *     Echem_stim_Init(&dac_module);
 *     
 *     // 启动GPCI实验
 *     GPCI_Electrochemistry_Example(&dac_module);
 *     
 *     // 主循环
 *     while (1) {
 *         // 检查是否需要填充缓冲区
 *         if (GPCI_NeedFillBuffer()) {
 *             GPCI_Fill_Next_Buffer();
 *         }
 *         
 *         // 其他任务...
 *         
 *         // 可选：检查实验进度
 *         const EchemResult_t *result = Echem_GetResult();
 *         if (result->state == ECHEM_STATE_COMPLETED) {
 *             // 实验完成，处理结果
 *             break;
 *         }
 *     }
 *     
 *     return 0;
 * }
 */
