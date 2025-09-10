#include "main.h"
#include "dma.h"
// #include "spi.h"
#include "gpio.h"
#include "LED.h"
#include "DAC80004.h"
#include "DDS_DAC80004.h"
#include "main_init.h"

uint16_t wave_high_data[1024*20];
uint16_t wave_low_data[1024*20];
uint16_t wave_buffer[1024*20];


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

  DDS_Init(&DAC80004_Module1);
  
  // DDS_Start_Precise(wave_data, 4, 10);
  // DDS_Start_Repeat(wave_data, 4, 10, 3);
 
  // bool success = Generate_And_Start_Sine_DDS(&DAC80004_Module1,
  //                                           1000.0,      // 目标频率 
  //                                           1000000.0,   // 最大采样率 
  //                                           20,          // 最小点数
  //                                           1024*20,     // 最大点数
  //                                           20000,       // 幅值
  //                                           32768,       // 偏移
  //                                           0,           // 重复次数 (0=无限循环)
  //                                           wave_buffer,
  //                                           wave_high_data,
  //                                           wave_low_data);
bool success = Generate_And_Start_CV_DDS(&DAC80004_Module1,
                                            0.0,        // 初始电位 0mV
                                            100.0,      // 终止电位 100mV
                                            -500.0,     // 扫描极限1 -500mV
                                            500.0,      // 扫描极限2 +500mV
                                            100.0,      // 扫描速率 100mV/s
                                            1000000.0,  // 最大采样率 1MHz
                                            100,        // 最小点数
                                            1024*20,       // 最大点数
                                            0,          // 重复3次
                                            wave_buffer,
                                            wave_high_data,
                                            wave_low_data);

  // for(uint16_t i=0;i<10;i++)
  //   {
  //     DAC80004_Data_Set(&DAC80004_Module1, 0xFFFF>>1);
  //     DAC80004_WriteData(&DAC80004_Module1);
        
  //   }
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
