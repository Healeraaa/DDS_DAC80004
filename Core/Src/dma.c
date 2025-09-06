/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */

void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  // NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  // NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream5_IRQn);

   NVIC_SetPriority(DMA2_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}
// void DMA1_Stream2_IRQHandler(void)
// {
//     if(LL_DMA_IsActiveFlag_TC2(DMA1))
//     {
//         // 传输完成，清除标志
//         LL_DMA_ClearFlag_TC2(DMA1);
        
//         // 如果需要继续传输，重新配置并启动 DMA
//         // LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
//         // LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, data_size);
//         // LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_2, (uint32_t)wave_data);
//         // LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
//     }
// }
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

