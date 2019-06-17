#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

void Modbus_Config(uint32_t baud);
void usart_putchar(uint8_t d);
void do_modbus(void);

#endif
