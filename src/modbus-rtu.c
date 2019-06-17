#include "stm32f10x.h"
#include "gpio_lib.h"
#include "modbus-rtu.h"
//#include "timer.h"
//#include "setup.h"
#include "crc_table.h"

#define MB_PACKET_SIZE				(260)
#define MB_BUF_SIZE_IN				(16)
#define MB_BUF_SIZE_OUT				(16)
#define MB_BUF_SIZE_3x				(256)
#define MB_BUF_SIZE_4x				(256)

#define DEV_ADDR					(0x01)

#define Tx_en()						(GPIOC->BSRR = (1<<12))
#define Tx_dis()					(GPIOC->BRR = (1<<12))

typedef struct
{
	uint8_t state;
	uint16_t pos;
	uint16_t len;
	uint8_t buf[MB_PACKET_SIZE];
} Modbus;

volatile Modbus mb;
volatile uint16_t buf_in[MB_BUF_SIZE_IN],buf_out[MB_BUF_SIZE_OUT],buf_3x[MB_BUF_SIZE_3x],buf_4x[MB_BUF_SIZE_4x];

extern void Set_link(uint32_t );

void usart_putchar(uint8_t d)
{
	Tx_en();
	while(!(USART3->SR & USART_SR_TXE));
	USART3->DR = d;
	while(!(USART3->SR & USART_SR_TC));
	Tx_dis();
}

void usart_puts(uint8_t *str)
{
	while(*str)
		usart_putchar(*str++);
}

enum mbs { MBS_START=0, MBS_WAIT, MBS_READY, MBS_SEND, MBS_WAIT_SEND };

//		Ќастройка USART дл€ работы с Modbus
//		PC10 - TXD3
//		PC11 - RXD3
//		PC12 - TXEN
//
void Modbus_Config(uint32_t baud)
{
	uint32_t brr;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	
	GPIO_PinConfig(GPIOC, 10, GPIO_MODE_AF10_PP);					// PC10 (TxD)  - AF 10MHZ mode
	GPIO_PinConfig(GPIOC, 11, GPIO_MODE_IF);						// PC11 (RxD)  - IF mode
	GPIO_PinConfig(GPIOC, 12, GPIO_MODE_GP10_PP);					// PC12 (TXEN) - GP 10MHz mode
	
	AFIO->MAPR &= ~(AFIO_MAPR_USART3_REMAP);
	AFIO->MAPR |= (AFIO_MAPR_USART3_REMAP_PARTIALREMAP);			//PC10 - TxD, PC11 - RxD, PC12 - CK ...

	DMA1_Channel2->CCR = DMA_CCR2_MINC|DMA_CCR2_DIR;
	DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel2->CMAR = (uint32_t)&mb.buf;
	
	DMA1_Channel3->CCR = DMA_CCR3_MINC;
	DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel3->CMAR = (uint32_t)&mb.buf;

	
	SystemCoreClockUpdate();
	USART3->CR2 = 0;
	USART3->CR3 = 0;
	
	brr = (uint32_t)(SystemCoreClock/2/baud); 
	
	USART3->BRR = brr;
	USART3->CR1 = USART_CR1_TE|USART_CR1_RE|USART_CR1_UE|USART_CR1_RXNEIE;
	USART3->CR3 = USART_CR3_DMAR|USART_CR3_DMAT;

	mb.state = MBS_START;												// автомат в режим ожидани€ данных
}
//      ѕодсчет CRC16
//
uint16_t do_crc16(volatile uint8_t *buf, uint16_t len)
{
	uint8_t crc_lo = 0xff;
	uint8_t crc_hi = 0xff;
	uint16_t i;
	while(len--)
	{
		i = crc_lo ^ (*buf++);
		crc_lo = (uint8_t)(crc_hi ^ crc_hi_table[i]);
		crc_hi = crc_lo_table[i];
	}
	return (uint16_t)((crc_hi<<8)|(crc_lo));
}
//      Prepare and send answer for CMD01 (Read coil status)
//      mb.buf:
//      - 0x02,0x03 - start adress (high,low bytes)
//      - 0x04,0x05 - number of coils (highm low bytes)
//      Answer:
//      0x00 - dev_addr
//      0x01 - 0x01 (CMD01)
//      0x02 - number of bytes (n)
//      0x03 - 1-st byte
//      ...	 - n-st byte
//      ...  - CRC (lo)
//      ...  - CRC (hi)
//
void do_cmd01(void)
{
    uint16_t addr,caddr,len,i,cnt_d;
    uint8_t d;

    addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];                // Coil(s) number
    addr = (addr>>3);                                           // Adress of coil in buffer
    len = (uint16_t)(mb.buf[4]<<8) + mb.buf[5];                 // Lenght 
    cnt_d = ((len-1)>>3)+1;                                     // Count of bytes with coil(s) values

    if (cnt_d >= MB_BUF_SIZE_OUT)
        cnt_d = MB_BUF_SIZE_OUT;

    for (i=0; i!=cnt_d; i++)
    {
        caddr = addr + i;                                       
        if (caddr >= MB_BUF_SIZE_OUT)
            d = 0;
        else
            d = buf_out[caddr];
        mb.buf[0x03 + i] = d;
    }
    mb.buf[0x02] = cnt_d;
    mb.len = cnt_d + 3;
}
//      Prepare ans send answer for CMD02
//		frame format:
//      	00 - target_addr
//      	01 - CMD02 (0x02)
//      	02 - start address, high byte
//      	03 - start address, low byte
//      	04 - number of coils fo read, high byte
//      	05 - number of coils fo read, low byte
//		answer format:
//			00 - dev_addr
//			01 - CMD02 (0x02)
//			02 - bytes count
//			03 - byte answer 1
//			.. - byte answer n
//			.. - CRC low byte
//			.. - CRC high byte
//
void do_cmd02(void)
{
    uint16_t addr,caddr,len,i;
    uint16_t cnt_d,d;
    addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];
    addr = (addr>>3);
    len = (uint16_t)(mb.buf[4]<<8) + mb.buf[5];

    cnt_d = ((len-1)>>3)+1;                                     // Number of bytes in answer frame
    if (cnt_d >= (MB_BUF_SIZE_IN-1))
        cnt_d = (MB_BUF_SIZE_IN-1);
    for (i=0; i!=cnt_d; i++)
    {
        caddr = addr + i;                                       
        if (caddr >= MB_BUF_SIZE_IN)
            d = 0;
        else
            d = buf_in[caddr];
        mb.buf[0x03 + i] = d;
    }
    mb.buf[0x02] = cnt_d;
    mb.len = cnt_d + 3;
}
//      Prepare ans send answer for CMD03 (Reading 4x register)
//		frame format:
//      	00 - target_addr
//      	01 - CMD03 (0x03)
//      	02 - start address 4x, high byte
//      	03 - start address 4x, low byte
//      	04 - number of registers for read, high byte
//      	05 - number of registers for read, low byte
//		answer format:
//			00 - dev_addr
//			01 - CMD03 (0x03)
//			02 - bytes count
//			03 - reg1 value, high byte
//			04 - reg1 value, low byte
//			.. - regN vaule, high byte
//			.. - regN value, low byte
//			.. - CRC low byte
//			.. - CRC high byte
//
void do_cmd03(void)
{
    uint16_t addr,caddr,cnt,d,i;

    addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];                // ????????? ?????
    cnt = (uint16_t)(mb.buf[4]<<8) + mb.buf[5];                 // ???-?? ?????? (????)

    if ((addr+cnt) >= MB_BUF_SIZE_4x)
        cnt = MB_BUF_SIZE_4x - addr;

    for (i=0; i!=cnt; i++)
    {
        caddr = addr + i;
        if (caddr >= MB_BUF_SIZE_4x)
            d = 0;
        else
            d = buf_4x[caddr];
        mb.buf[0x03 + (i*2)] = d>>8;
        mb.buf[0x04 + (i*2)] = d&0xff;
    }
    mb.buf[0x02] = cnt<<1;
    mb.len = (cnt<<1) + 3;
}
//		Prepare ans send answer for CMD04 (Reading 3x register)
//		frame format:
//      	00 - target_addr
//      	01 - CMD04 (0x04)
//      	02 - start address 4x, high byte
//      	03 - start address 4x, low byte
//      	04 - number of registers for read, high byte
//      	05 - number of registers for read, low byte
//		answer format:
//			00 - dev_addr
//			01 - CMD04 (0x04)
//			02 - bytes count
//			03 - reg1 value, high byte
//			04 - reg1 value, low byte
//			.. - regN vaule, high byte
//			.. - regN value, low byte
//			.. - CRC low byte
//			.. - CRC high byte
//
void do_cmd04(void)
{
    uint16_t caddr,addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];                // ????????? ?????
    uint16_t i,d,cnt = (uint16_t)(mb.buf[4]<<8) + mb.buf[5];                 // ???-?? ?????? (????)

    if ((addr+cnt) >= MB_BUF_SIZE_3x)
        cnt = MB_BUF_SIZE_3x - addr;
    for (i=0; i!=cnt; i++)
    {
        caddr = addr + i;
        if (caddr >= MB_BUF_SIZE_3x)
            d = 0;
        else
            d = buf_3x[caddr];
        mb.buf[0x03 + (i*2)] = d>>8;
        mb.buf[0x04 + (i*2)] = d&0xff;
    }
    mb.buf[0x02] = cnt<<1;
    mb.len = (cnt<<1) + 3;
}

//		Prepare ans send answer for CMD05 (Write coil)
//		frame format:
//			00 - target_adr
//			01 - CMD05 (0x05)
//			02 - address of coil, high byte
//			03 - address of coil, low byte
//			04 - 0xff for set or 0x00 to reset coil
//		answer format:
//			equal to frame format
//
void do_cmd05(void)
{
    uint16_t addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];                       // ????? ??????
    uint16_t caddr = (addr>>3);                                                 // ????? ?????, ? ??????? ????? ??????
    uint8_t n = addr-(caddr<<3);                                                // ????? ???? (0-7)
	mb.len = 0;
    if (caddr < MB_BUF_SIZE_OUT)
    {
        if (mb.buf[0x04] == 0xff)
            buf_out[caddr] |= (1<<n);
        else if (mb.buf[0x04] == 0x00)
            buf_out[caddr] &= ~(1<<n);
        mb.len = 6;
    }
}
//		Prepare ans send answer for CMD06 (write 4x register)
//		frame format:
//			00 - target_adr
//			01 - CMD05 (0x06)
//			02 - address of register, high byte
//			03 - address of register, low byte
//			04 - value, high byte
//			05 - value, low byte
//		answer format:
//			equal to frame format
//
void do_cmd06(void)
{
    uint16_t addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];                       // ????? ????????
    uint16_t val = (uint16_t)(mb.buf[4]<<8) + mb.buf[5];
	mb.len = 0;
    if (addr < MB_BUF_SIZE_4x)
    {
        buf_4x[addr] = val;
        mb.len = 6;
    }
}
//		Prepare ans send answer for CMD16 (write 4x registers)
//		frame format:
//			00 - target_adr
//			01 - CMD16 (0x10)
//			02 - start address of register, high byte
//			03 - start address of register, low byte
//			04 - number of registers, high byte
//			05 - number of registers, low byte
//			06 - bytes count (N*2)
//			07 - value 1, high byte
//			08 - value 1, low byte		
//			.. - value N, high byte
//			.. - value N, low byte
//		answer format:
//			00 - dev_addr
//			01 - CMD16 (0x10)
//			02 - start adress of register, high byte
//			03 - start address of register, low byte
//			04 - number of registers, high byte
//			05 - number of registers, low byte
//			06 - CRC, low byte
//			07 - CRC, high byte
//
void do_cmd16(void)
{
    uint16_t addr = (uint16_t)(mb.buf[2]<<8) + mb.buf[3];                       // Start address
    uint16_t cnt  = (uint16_t)(mb.buf[4]<<8) + mb.buf[5];                       // Number of registers
    uint16_t i,val;

	mb.len = 0;
	
    if ((addr+cnt) >= MB_BUF_SIZE_4x)
        cnt = MB_BUF_SIZE_4x - addr;
    for (i=0; i!=cnt; i++)
    {
        val = (uint16_t)(mb.buf[7+(i*2)]<<8) + mb.buf[8+(i*2)];
        buf_4x[addr+i] = val;
    }
    if (cnt)
    {
        mb.len = 6;
    }
}
void Modbus_parse(void)
{
	switch (mb.buf[1])
	{
		case 01 : do_cmd01(); break;
		case 02 : do_cmd02(); break;
		case 03 : do_cmd03(); break;
		case 04 : do_cmd04(); break;
		case 05 : do_cmd05(); break;
		case 06 : do_cmd06(); break;
		case 16 : do_cmd16(); break;
	}
}

void do_modbus(void)
{
	switch(mb.state)
	{
		case MBS_START:														// Ќастраиваем DMA на прием
		{
			(USART3->DR);
			DMA1->IFCR |= DMA_IFCR_CTCIF3|DMA_IFCR_CGIF3|DMA_IFCR_CHTIF3|DMA_IFCR_CTEIF3;
			DMA1_Channel3->CNDTR = 255;
			DMA1_Channel3->CCR |= DMA_CCR3_EN;
			
			mb.len = mb.pos = 0;
			mb.state = MBS_WAIT;
			break;
		}
		case MBS_WAIT:
		{
			if (USART3->SR & USART_SR_IDLE)									// при приеме возникла пауза на линии:
			{
				mb.len = 255 - DMA1_Channel3->CNDTR;						// - подсчитаем кол-во прин€тых данных
				DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
				mb.state = MBS_READY;
			}
			break;
		}
		case MBS_READY:
		{
			if (mb.len && !(do_crc16(mb.buf,mb.len)))						// провер€ем наличие данных и их CRC
			{
				if (mb.buf[0] == DEV_ADDR)									// адрес устройства наш?
				{
					Modbus_parse();											// - пытаемс€ отработать прин€тый пакет
					Set_link(10);											// - мигнем светодиодом
					mb.state = MBS_SEND;									// - отправим ответ
				}
			} else
			{
				mb.state = MBS_START;										// иначе снова на прием
			}
			break;
		}
		case MBS_SEND:
		{
			uint16_t crc;
			if (mb.len)														// есть данные дл€ отправки:
			{
				crc = do_crc16(mb.buf,mb.len);
				mb.buf[mb.len] = crc&0xff;									// дополним пакет его CRC
				mb.buf[mb.len+1] = (crc>>8)&0xff;
				DMA1->IFCR |= DMA_IFCR_CTCIF2|DMA_IFCR_CGIF2|DMA_IFCR_CHTIF2|DMA_IFCR_CTEIF2;
				DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
				DMA1_Channel2->CNDTR = mb.len + 2;
				Tx_en();													// переключимс€ на отправку
				DMA1_Channel2->CCR |= DMA_CCR2_EN;							// отправл€ем пакет
				mb.state = MBS_WAIT_SEND;
			} else
				mb.state = MBS_START;
			break;
		}
		case MBS_WAIT_SEND:
		{
			if ((DMA1->ISR & DMA_ISR_TCIF2) && (USART3->SR & USART_SR_TC))	// пакет отправлен:
			{
				Tx_dis();													// переключимс€ на прием
				mb.state = MBS_START;										// переход в ожидание нового пакета
			}
			break;
		}
	}
}
