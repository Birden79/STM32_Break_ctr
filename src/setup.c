#include "stm32f10x.h"
#include "gpio_lib.h"

//	Configure IO
//	PA5 	- KEY		Input floating
//	PA8 	- OSC25		AF, 50MHz
//	PA11	- LED2		GP Output
//	PA12	- LED1		GP Output
//	
//	PC6		- ENCA
//	PC7		- ENCB
//	PC8		- ENCZ
//
//	PB14	- BREN		Input floating
//	PB15	- ROUT		GP Output
//
void IO_Config(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN|RCC_APB2ENR_AFIOEN;
	
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG);
	AFIO->MAPR |= (AFIO_MAPR_SWJ_CFG_JTAGDISABLE);			// Only SWD enable
	
	GPIO_PinConfig(GPIOA, 5, GPIO_MODE_IF);					// KEY
	GPIO_PinConfig(GPIOA, 8, GPIO_MODE_AF50_PP);
	GPIO_PinConfig(GPIOA, 11, GPIO_MODE_GP2_PP);
	GPIO_PinConfig(GPIOA, 12, GPIO_MODE_GP2_PP);
	
	GPIO_PinConfig(GPIOB, 14, GPIO_MODE_IF);
	GPIO_PinConfig(GPIOB, 15, GPIO_MODE_GP2_PP);
	
	RCC->CFGR &= ~(RCC_CFGR_MCO);
	RCC->CFGR |= RCC_CFGR_MCO_3;
}
