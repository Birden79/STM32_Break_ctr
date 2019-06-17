#include "stm32f10x.h"
#include "encoder.h"
#include "gpio_lib.h"

static int16_t cur_rpm, old_rpm = 0, cur_accel;

//		Configure PC6, PC7 as encoder input
//
void Encoder_config(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	GPIO_PinConfig(GPIOC, 6, GPIO_MODE_IF);
	GPIO_PinConfig(GPIOC, 7, GPIO_MODE_IF);
	
	AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP;
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_FULLREMAP;				// TIM3 Full remap: CH1-PC6, CH2-PC7
	
	TIM3->CCMR1 = TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0;
	TIM3->CCER = TIM_CCER_CC1E|TIM_CCER_CC2E;
	TIM3->SMCR = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1;
	TIM3->ARR = 0xffff;
	TIM3->CR1 = TIM_CR1_CEN;
}
//		Расчет скорости и ускорения
//		enc_value - скорость в единицах энкодера (положение)
//		ppr_value - кол-во импульсов энкодера на оборот
//		freq - частота измерений, Гц
//
void Encoder_check(int16_t enc_value, uint16_t ppr_value, uint16_t freq)
{
	cur_rpm = (int32_t)(enc_value*60*freq)/ppr_value;
	cur_accel = (cur_rpm - old_rpm)*freq/60;
	old_rpm = cur_rpm;
}

int16_t GetCurrentRPM(void)
{
	return cur_rpm;
}
int16_t GetCurrentAccel(void)
{
	return cur_accel;
}
void SetEncPPR(uint32_t ppr)
{
}
