#include "main.h"
#include "dma.h"
// #include "spi.h"
#include "adc.h"
#include "gpio.h"
#include "LED.h"
#include "DAC80004.h"
#include "AD5231.h"
// #include "DDS_DAC80004.h"
#include "Echem_stim.h"
#include "main_init.h"
#include "usart.h"
#include "Serial.h"
#include "Serial_Process.h"


uint8_t dma_cnt = 0;
uint8_t dma1_cnt = 0;
uint8_t dma2_cnt = 0;

Serial_DoubleConverter_t Serial_Data[2] ;
double rx_data[2] = {0};

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
  USART1_Init();
  LL_mDelay(500);
  LED_OFF();

  // //数字电位器测试开始
  // // SPI2_Init();
  // AD5231_Init();
  // AD5231_WriteRDAC(512);
  // ADC1_Init();
  // volatile uint32_t adc_value = 0;
  // volatile float f_adc_value = 0;
  // while (1)
  // {
    
  //   // SPI_Transmit8_Time(SPI2, 0xA5, 1); // 发送命令字节
  //   AD5231_WriteRDAC(0);
  //   adc_value = ADC1_FifterRead();
  //   f_adc_value = (float)adc_value * 3.32f / 4095.0f;
  //   LED_Reveral();
  //   LL_mDelay(500);
  // }
  // //数字电位器测试结束

  Echem_stim_Init(&DAC80004_Module1);

  // double data[10] = {100.0,-800.0,50.0,75.0,250.0,500.0,0,0.0,0.0,0.0};

  // Serial_DPV_CreateWave(data);
 
  

  while (1)
  {
    Serial_Process();
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
