#include "main_init.h"
#include "Echem_stim.h"

dac80004_bus_t DAC80004_dev1 =
{
    .LDAC_PORT = GPIOA,
    .CLR_PORT = GPIOA,
    .CS_PORT = GPIOA,
    .LDAC_PIN = LL_GPIO_PIN_2,
    .CLR_PIN = LL_GPIO_PIN_3,
    .CS_PIN = LL_GPIO_PIN_4,
};

DAC80004_InitStruct DAC80004_Module1 =
{
    .dev = &DAC80004_dev1,
    .WR_Buff = 0,
    .Command_Buff = 0,
    .Channel_Buff = 0,
    .Data_Buff = 0,
    .Mode_Buff = 0,
    .TX_Data = 0,  
};

PingPongConfig_t config = {
    .buffer_size = 1024*8,         // 单个缓冲区大小
    .max_sample_rate = 85000.0, // 最大采样率 1MHz
    .min_points = 100,           // 最小点数
    .max_points = 1024*8,       // 最大点数
    .enable_progress_callback = true,
    .enable_error_recovery = false
};


uint16_t wave_high_data1[1024*8];
uint16_t wave_high_data2[1024*8];
uint16_t wave_low_data1[1024*8];
uint16_t wave_low_data2[1024*8];

void Main_Variables_Init(void)
{
    memset(wave_high_data1, 0, sizeof(wave_high_data1));
    memset(wave_high_data2, 0, sizeof(wave_high_data2));
    memset(wave_low_data1, 0, sizeof(wave_low_data1));
    memset(wave_low_data2, 0, sizeof(wave_low_data2));
}


