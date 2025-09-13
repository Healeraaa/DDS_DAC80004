#include "main.h"
#include "dma.h"
// #include "spi.h"
#include "gpio.h"
#include "LED.h"
#include "DAC80004.h"
// #include "DDS_DAC80004.h"
#include "Echem_stim.h"
#include "main_init.h"

// uint16_t wave_high_data[1024*20];
// uint16_t wave_low_data[1024*20];
// uint16_t wave_buffer[1024*20];

uint16_t wave_high_data1[1024*8];
uint16_t wave_high_data2[1024*8];
uint16_t wave_low_data1[1024*8];
uint16_t wave_low_data2[1024*8];

uint8_t dma_cnt = 0;
uint8_t dma1_cnt = 0;
uint8_t dma2_cnt = 0;

void SystemClock_Config(void);
uint32_t data32 = 0;
int main(void)
{

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  SystemClock_Config();

  System_GPIO_Init();

  LED_Init();
  LED_ON();
  LL_mDelay(1000);

  // DDS_Init(&DAC80004_Module1);
  Echem_stim_Init(&DAC80004_Module1);

  EchemCV_Params_t cv_params = {
        // 基本电位参数
        .Initial_E = 500.0,      // 初始电位 500mV
        .Final_E = -500.0,        // 终止电位 500mV  
        .Scan_Limit1 = -1000.0,   // 扫描极限1 -500mV
        .Scan_Limit2 = 1000.0,    // 扫描极限2 500mV
        .Scan_Rate = 1000.0,      // 扫描速率 100mV/s
        .cycles = 2,             // 循环3次
        .equilibrium_time = 2.0, // 平衡时间 2s
        .auto_sensitivity = true,
        
        // 以下字段会由系统自动计算填充，无需手动设置
        .initial_points = 0,
        .cycle_points = 0,
        .final_points = 0,
        .total_points = 0,
        .actual_sample_rate = 0.0
    };
    
    // 乒乓DMA配置
    PingPongConfig_t config = {
        .buffer_size = 1024*8,         // 单个缓冲区大小
        .max_sample_rate = 85000.0, // 最大采样率 1MHz
        .min_points = 100,           // 最小点数
        .max_points = 1024*8,       // 最大点数
        .enable_progress_callback = true,
        .enable_error_recovery = false
    };
    CV_DDS_Start_Precise(&DAC80004_Module1, 
                          &cv_params, 
                          &config, 
                          wave_high_data1,
                          wave_high_data2,
                          wave_low_data1, 
                          wave_low_data2);


    while (!PingPong_DMA_IsComplete()) 
    {
      // 检查是否需要填充缓冲区（关键！）
      if (CV_NeedFillBuffer()) {
        dma_cnt++;
      
        CV_Fill_Next_Buffer();
      }
        }

        LL_mDelay(1000);
    CV_DDS_Start_Precise(&DAC80004_Module1, 
                          &cv_params, 
                          &config, 
                          wave_high_data1,
                          wave_high_data2,
                          wave_low_data1, 
                          wave_low_data2);


    while (!PingPong_DMA_IsComplete()) 
    {
      // 检查是否需要填充缓冲区（关键！）
      if (CV_NeedFillBuffer()) {
        dma_cnt++;
      
        CV_Fill_Next_Buffer();
      }
        }


    while (1)
    {
			


        LL_mDelay(100);
    

    // SPI1_Transmit16_Time(0xA55A, 1);
    // DAC80004_Channel_Config(&DAC80004_Module1, DAC_CH_A);
    // DAC80004_Data_Set(&DAC80004_Module1, 0);
    // DAC80004_WriteData(&DAC80004_Module1);
    // LL_mDelay(100);
  }
}


/**
 * @brief System Clock Configuration
 * @retval None

 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_12, 96, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1)
  {
  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  LL_Init1msTick(100000000);
  LL_SetSystemCoreClock(100000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
