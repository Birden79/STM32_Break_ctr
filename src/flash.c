#include "stm32f10x.h"
#include "flash.h"

#define FLASH_PAGE_NUM				(63)
#define FLASH_PAGE_SIZE				(1024)
#define FLASH_PAGE_ADDR				(0x08000000 + (uint32_t)FLASH_PAGE_NUM*FLASH_PAGE_SIZE)
#define FLASH_KEY1 					((uint32_t)0x45670123)
#define FLASH_KEY2 					((uint32_t)0xCDEF89AB)

uint32_t FlashRead32(uint32_t addr)
{
	return (*(__IO uint32_t *)addr);
}
uint16_t FlashRead16(uint32_t addr)
{
	return (*(__IO uint16_t *)addr);
}

void FlashReadBuf(uint32_t addr, uint32_t cnt)
{
	uint32_t flash_addr = FLASH_PAGE_ADDR;
	uint16_t *buf = (uint16_t *)addr;
	while(cnt--)
	{
		*buf++ = FlashRead16(flash_addr);
		flash_addr += 2;
	}
}
void FlashUnlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;	
}
void FlashLock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}
uint8_t FlashIsReady(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}
void FlashErasePage(uint32_t page_addr)
{
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = page_addr;
	FLASH->CR |= FLASH_CR_STRT;
	while(!FlashIsReady());
	FLASH->CR &= ~FLASH_CR_PER;
}
void FlashWrite16(uint32_t addr, uint16_t data)
{
	FLASH->CR |= FLASH_CR_PG;
	while(!FlashIsReady());
	*(__IO uint16_t *)addr = data;
	while(!FlashIsReady());
	FLASH->CR &= ~FLASH_CR_PG;
}
void FlashWriteBuf(uint32_t addr, uint16_t cnt)
{
	uint16_t *data = (uint16_t *)addr;
	uint32_t flash_addr = FLASH_PAGE_ADDR;
	FlashUnlock();
	FlashErasePage(FLASH_PAGE_ADDR);
	while(cnt--)
	{
		FlashWrite16(flash_addr, *data++);
		flash_addr += 2;
	}
	FlashLock();
}
