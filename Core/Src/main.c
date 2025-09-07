#include "main.h"
#include "dma.h"
// #include "spi.h"
#include "gpio.h"
#include "LED.h"
#include "DAC80004.h"
#include "DDS_DAC80004.h"
#include "main_init.h"

uint16_t wave_high_data[4096];
uint16_t wave_low_data[4096];
uint16_t wave_buffer[4096];


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
  SineWaveResult_t result;
  Generate_Smart_Sine_Wave(wave_buffer, 50.0, 2000.0, 20, 4096, 20000, 32768, &result);
  Encode_Wave_DualDMA(&DAC80004_Module1, wave_buffer,wave_high_data,wave_low_data, result.points);
  // DDS_Start_Repeat(&DAC80004_Module1,wave_data, result.points*2, result.actual_sample_rate, 0);
  DDS_Start_DualDMA(&DAC80004_Module1,wave_high_data, wave_low_data, result.points, result.points, result.actual_sample_rate, 0);
    
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
