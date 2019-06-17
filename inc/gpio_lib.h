#pragma once

#define GPIO_MODE_ANALOG		(0x00)		// 0000
#define GPIO_MODE_IF			(0x04)		// 0100
#define GPIO_MODE_IPUPD			(0x08)		// 1000
#define GPIO_MODE_GP10_PP		(0x01)		// 0001
#define GPIO_MODE_GP10_OD		(0x05)		// 0101
#define GPIO_MODE_AF10_PP		(0x09)		// 1001
#define GPIO_MODE_AF10_OD		(0x0d)		// 1101
#define GPIO_MODE_GP2_PP		(0x02)		// 0010
#define GPIO_MODE_GP2_OD		(0x06)		// 0110
#define GPIO_MODE_AF2_PP		(0x0a)		// 1010
#define GPIO_MODE_AF2_OD		(0x0e)		// 1110
#define GPIO_MODE_GP50_PP		(0x03)		// 0011
#define GPIO_MODE_GP50_OD		(0x07)		// 0111
#define GPIO_MODE_AF50_PP		(0x0b)		// 1011
#define GPIO_MODE_AF50_OD		(0x0f)		// 1111

void GPIO_PinConfig(GPIO_TypeDef *GPIO, uint32_t pin, uint32_t mode);
