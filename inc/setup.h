#pragma once

extern volatile uint16_t buf_3x[],buf_4x[];

#define LED1_PIN				(12)
#define LED2_PIN				(11)
#define KEY_PIN					(5)
#define BREN_PIN				(14)
#define RELAY_PIN				(15)
#define LED_PORT				GPIOA
#define KEY_PORT				GPIOA
#define BREN_PORT				GPIOB
#define RELAY_PORT				GPIOB
#define LED1_on()				(LED_PORT->BSRR = (1<<LED1_PIN))
#define LED1_off()				(LED_PORT->BRR  = (1<<LED1_PIN))
#define LED2_on()				(LED_PORT->BSRR = (1<<LED2_PIN))
#define LED2_off()				(LED_PORT->BRR  = (1<<LED2_PIN))
#define RELAY_on()				(RELAY_PORT->BSRR = (1<<RELAY_PIN))
#define RELAY_off()				(RELAY_PORT->BRR = (1<<RELAY_PIN))
#define BR_isEnable()			(BREN_PORT->IDR & (1<<BREN_PIN))
#define KEY_in()				(~(KEY_PORT->IDR) & (1<<KEY_PIN))

#define ext_status				(buf_3x[0])
#define ext_enc_rpm				(buf_3x[1])
#define ext_enc_accel			(buf_3x[2])
#define ext_fixed_accel			(buf_3x[3])
#define ext_rpm_buf				(buf_3x[4])

#define ext_cmd					(buf_4x[0])
#define ext_max_accel			(buf_4x[1])
#define ext_enc_rpr				(buf_4x[2])

#define EXT_STATUS_ALARM		(0x0001)
#define EXT_STATUS_BR_ENABLE	(0x0002)

#define EXT_CMD_RESET_ALARM		(0x0001)
#define EXT_CMD_SAVE			(0x0002)
#define EXT_CMD_LOAD			(0x0004)

void IO_Config(void);
void SetSycClock(void);
