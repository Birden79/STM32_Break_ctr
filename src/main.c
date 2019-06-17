/*
	Use git
*/
#include "stm32f10x.h"
#include "gpio_lib.h"
#include "setup.h"
#include "modbus-rtu.h"
#include "encoder.h"
#include "flash.h"

static uint32_t tcnt,led_cnt,flags;
static uint16_t keyb, kcnt;

#define FL_SEC					(0x0001)
#define FL_CHECK				(0x0002)
#define FL_ACCEL_ALARM			(0x0008)

#define BUF_SIZE		(100)
#define CHECK_FREQ		(100)			// Частота измерений, Гц

void BufStore(uint16_t val)
{
	uint8_t i;
	for (i=BUF_SIZE-1; i>0; i--)
		buf_3x[4+i] = buf_3x[4+i-1];
	ext_rpm_buf = val;
}

uint16_t abs_value(int16_t n)
{
	if (n < 0) n = -n;
	return (uint16_t)n;
}

void CheckAlarm(void)
{
	if (!(flags & FL_ACCEL_ALARM) && BR_isEnable() && (abs_value((int16_t)ext_enc_accel) > (int16_t)ext_max_accel))
	{
		flags |= FL_ACCEL_ALARM;
		ext_fixed_accel = ext_enc_accel;
	}
}

void SysTick_Handler(void)
{
	int16_t enc_value;
	if (++tcnt == (1000/CHECK_FREQ))
	{
		tcnt = 0;
		flags |= FL_CHECK;
		enc_value = (int16_t)TIM3->CNT;
		TIM3->CNT = 0;

		Encoder_check(enc_value, ext_enc_rpr, CHECK_FREQ);

		ext_enc_rpm = GetCurrentRPM();
		ext_enc_accel = GetCurrentAccel();
		BufStore(ext_enc_rpm);
		
		CheckAlarm();
	}
	
	if (led_cnt) { led_cnt--; LED1_on(); } else LED1_off();
	
	if (KEY_in()) 
	{
		if (kcnt < 1000) kcnt++;
	} else
		kcnt=0;
	
	if (kcnt == 100)
	{
		keyb = 1;
	}
}
void Set_link(uint32_t n)
{
	led_cnt += n;
}
void ResetAlarm(void)
{
	flags &= ~FL_ACCEL_ALARM;
	ext_fixed_accel = 0;
}

void SaveAllSettings(void)
{
	FlashWriteBuf((uint32_t)&ext_max_accel, 2);
}
void LoadAllSettings(void)
{
	FlashReadBuf((uint32_t)&ext_max_accel, 2);
	if (!ext_enc_rpr || (ext_enc_rpr > 32000))
		ext_enc_rpr = 4096;
	if (!ext_max_accel || (ext_max_accel > 32000))
		ext_max_accel = 500;
}
/**

*/
int main(void)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	LoadAllSettings();
	
	IO_Config();
	Encoder_config();
	Modbus_Config(115200);
	while(1)
	{
		if (flags & FL_ACCEL_ALARM)
			RELAY_off();
		else
			RELAY_on();
		if (keyb == 1)
		{
			keyb = 0;
			ResetAlarm();
		}
		if (ext_cmd & EXT_CMD_RESET_ALARM)
		{
			ext_cmd &= ~EXT_CMD_RESET_ALARM;
			ResetAlarm();
		}
		if (ext_cmd & EXT_CMD_SAVE)
		{
			ext_cmd &= ~EXT_CMD_SAVE;
			SaveAllSettings();
		}
		if (ext_cmd & EXT_CMD_LOAD)
		{
			ext_cmd &= ~EXT_CMD_LOAD;
			LoadAllSettings();
		}
		if (flags & FL_ACCEL_ALARM) ext_status |= EXT_STATUS_ALARM; else ext_status &= ~EXT_STATUS_ALARM;
		if (BR_isEnable()) ext_status |= EXT_STATUS_BR_ENABLE; else ext_status &= ~EXT_STATUS_BR_ENABLE;
			
		do_modbus();
	}
}
