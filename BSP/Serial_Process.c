#include "Serial_Process.h"
#include "Serial.h"
#include "Echem_stim.h"
#include "main_init.h"
#include "LED.h"

double g_serial_rx_doubel_data[SERIAL_DATA_LENGTH] = {0};
uint8_t g_serial_rx_command = 0;




void Serial_CV_CreateWave(double *data)
{
    EchemCV_Params_t cv_params = {
        // 基本电位参数
        .Initial_E = data[0],      // 初始电位 
        .Final_E = data[1],        // 终止电位 
        .Scan_Limit1 = data[2],   // 扫描极限1 
        .Scan_Limit2 = data[3],    // 扫描极限2 
        .Scan_Rate = data[4],      // 扫描速率 
        .cycles = (uint32_t)data[5], // 循环n次
        .equilibrium_time = 2.0, // 平衡时间 2s
        .auto_sensitivity = true,
        // 以下字段会由系统自动计算填充，无需手动设置
        .initial_points = 0,
        .cycle_points = 0,
        .final_points = 0, 
        .total_points = 0,
        .actual_sample_rate = 0.0
    };
    CV_DDS_Start_Precise(&DAC80004_Module1, 
                          &cv_params, 
                          &config, 
                          wave_high_data1,
                          wave_high_data2,
                          wave_low_data1, 
                          wave_low_data2);
    LED_ON();
    while (!PingPong_DMA_IsComplete()) 
    {
      if (CV_NeedFillBuffer()) {
        CV_Fill_Next_Buffer();
      }
      }
}

void Serial_DPV_CreateWave(double *data)
{
    EchemDPV_Params_t dpv_params = {
        .Initial_E = data[0],        // 初始电位 
        .Final_E = data[1],           // 终止电位 
        .Step_E = data[2],             // 步进电位 
        .Pulse_Amplitude = data[3],    // 脉冲幅度 
        .Pulse_Width = data[4],        // 脉冲宽度    
        .Pulse_Period = data[5],      // 脉冲周期 
        .equilibrium_time = 2.0,    // 平衡时间 2s
        .auto_sensitivity = true,
        // 以下字段会由系统自动计算填充，无需手动设置
        .total_steps = 0,         // 总步数
        .points_per_step = 0,       // 每步的点数
        .pulse_points = 0,        // 脉冲部分点数
        .base_points = 0,       // 基础部分点数
        .total_points = 0,         // 总点数
        .actual_sample_rate = 0.0     // 实际采样率 (Hz)
    };
    DPV_DDS_Start_Precise(&DAC80004_Module1, &dpv_params, &config,
                             wave_high_data1, wave_high_data2,
                             wave_low_data1, wave_low_data2); 
    LED_ON();
    while (!PingPong_DMA_IsComplete()) 
    {
        if (DPV_NeedFillBuffer()) {
            DPV_Fill_Next_Buffer();
        }
        
    }
}
/**
 * @brief  串口命令处理函数
 * @param  method: 方法命令字
 */
void Serial_CommandTask(uint8_t method)
{
    switch (method)
    {
    case ECHEM_METHOD_CV:
        Serial_CV_CreateWave(g_serial_rx_doubel_data);
        break;
    case ECHEM_METHOD_DPV:
        Serial_DPV_CreateWave(g_serial_rx_doubel_data);
        break;
    default:
        break;
    }
}

/**
 * @brief  串口数据处理函数
 */
void Serial_Process(void)
{
    if(Serial_GetRxFlag()==1)
    {
      g_serial_rx_command = Serial_GetRxCommand();
      Serial_GetRxData(g_serial_rx_doubel_data,SERIAL_DATA_LENGTH);
      Serial_CommandTask(g_serial_rx_command);
      LED_OFF();
    }
}

