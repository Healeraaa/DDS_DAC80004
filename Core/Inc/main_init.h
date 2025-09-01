#ifndef __MAIN_INIT_H
#define __MAIN_INIT_H

#include "main.h"
#include "DAC80004.h"

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



#endif
