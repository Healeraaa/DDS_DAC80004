#include "LED.h"
#include "gpio.h"

void LED_Init(void)
{
	GPIOC_Group1_Init();
}

void LED_ON(void)
{
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
}

void LED_OFF(void)
{
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
}

void LED_Reveral(void)
{
	LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}
