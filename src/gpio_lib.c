#include "stm32f10x.h"
#include "gpio_lib.h"


void GPIO_PinConfig(GPIO_TypeDef *GPIO, uint32_t pin, uint32_t mode)
{
	pin &= 0x1f;
	mode &= 0x0f;
	if (pin < 8)
	{
		GPIO->CRL &= ~(0x0f << (pin*4));
		GPIO->CRL |= (mode << (pin*4));
	} else
	{
		pin -= 8;
		GPIO->CRH &= ~(0x0f << (pin*4));
		GPIO->CRH |= (mode << (pin*4));
	}
}
