#ifndef __SPI_H__
#define __SPI_H__


#include "main.h"

void SPI1_Init(void);
void SPI1_SetDataWidth(uint32_t dataWidth);
void SPI1_Transmit_DMA(uint8_t *data, uint32_t size);
void SPI1_Transmit8_Time(uint8_t data, uint32_t Timeout);
void SPI1_Transmit16_Time(uint16_t data, uint32_t Timeout);
void SPI1_Transmit_DMA_Start(uint8_t *data, uint32_t size);
uint8_t SPI1_Transmit_DMA_WaitComplete(void);

void SPI2_Init(void);
void SPI_Transmit8_Time(SPI_TypeDef *SPIx,uint8_t data, uint32_t Timeout);


#endif /* __SPI_H__ */

