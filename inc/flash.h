#pragma once

uint32_t FlashRead(uint32_t addr);
void FlashReadBuf(uint32_t addr, uint32_t cnt);
void FlashWriteBuf(uint32_t addr, uint16_t cnt);
