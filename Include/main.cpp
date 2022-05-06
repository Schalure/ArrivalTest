#define _MAIN_
/* Includes ------------------------------------------------------------------*/
#include "common.h"


//------------------------------------------------------------------------------
//	for UART
#define UART_DIV_SAMPLING16(_PCLK_, _BAUD_)            (((_PCLK_)*25U)/(4U*(_BAUD_)))
#define UART_DIVMANT_SAMPLING16(_PCLK_, _BAUD_)        (UART_DIV_SAMPLING16((_PCLK_), (_BAUD_))/100U)
#define UART_DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_)        (((UART_DIV_SAMPLING16((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100U)) * 16U + 50U) / 100U)

#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_)            (((UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4U) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0xF0U)) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0FU))
//------------------------------------------------------------------------------
void initialize();

//------------------------------------------------------------------------------
int main(void)
{
	__disable_interrupt();	
	initialize();

	__enable_interrupt();
		
	while (1)
	{
		//каждую миллисекнду
		if(u32_TickCountOld != u32_TickCount)
		{	
			u32_TickCountOld++;
			test_time_out();//контроль функций ожидающих запуска
		}
		run();	
	}
}

//------------------------------------------------------------------------------
void initialize()
{
	SystemCoreClockUpdate();
	Init_SysTick(1);
	
	
	//	UART1 initialize
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
	//	tx/rx pins init
	GPIOA->CRH &= (~GPIO_CRH_CNF9_0);
	GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

	GPIOA->CRH &= (~GPIO_CRH_CNF10_0);
	GPIOA->CRH |= GPIO_CRH_CNF10_1;
	GPIOA->CRH &= (~(GPIO_CRH_MODE10));
	GPIOA->BSRR |= GPIO_ODR_ODR10;

	// uart1 init
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
	USART1->BRR = UART_BRR_SAMPLING16(SystemCoreClock, 460800); // 460800 - target br
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	
	NVIC_EnableIRQ (USART1_IRQn);
	USART1->CR1 = USART_CR1_UE;
	//	END UART1 initialize


	//	TIMER initialize
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 |= TIM_CR1_ARPE;
	//	10 kHz (0xC80)
	TIM2->PSC = SystemCoreClock / 10000 - 1;
	//	TIM2->PSC * 3000 = 300 Hz (0xBB8)
	TIM2->ARR = 3000;
	
	//	start value, every tick for 10 kHz
	TIM2->CCR1 = 1;
	//	start value, every 1000 ticks for 100 Hz (adc triger)
	TIM2->CCR2 = 1000;
	//	start value, every 100 ticks for 1 kHz
	TIM2->CCR3 = 100;
	//	start value, every 3000 ticks for 300 Hz
	TIM2->CCR4 = 3000;

	//	interrupts
	TIM2->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
	
	//	start timer
	TIM2->CR1 |= TIM_CR1_CEN;
	//	END TIMER initialize
	
	//	GPIOB initialize, enabled clock & default settings
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	//	END GPIOB initialize

	
	//	ADC initialize
	//	initialize PA0 to analog
	GPIOA->CRL &= ~ (GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	//	choose the ADC clk divider
	u8  divider = 2;
	for(; divider <= 8; divider += 2)
	{
		if(SystemCoreClock / divider <= 14000000)
			break;
		else
			divider = 8;
	}
	RCC->CFGR |= divider << 14;
	
	//	interrupt en
	ADC1->CR1 |= ADC_CR1_EOCIE;
	NVIC_EnableIRQ (ADC1_2_IRQn); 
	//	adc en, ext triger (TIM2 CC2)
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1;
	for(u16 i = 0; i < 10000; ++i);
	//	start calibration
	ADC1->CR2 |= ADC_CR2_CAL;
	while((ADC1->CR2 & ADC_CR2_CAL) != 0);
	//	END ADC initialize
	
}

//------------------------------------------------------------------------------
//														UART EHO
//------------------------------------------------------------------------------
#define	_FALSE_	0
#define	_TRUE_	1
#define	_CRC_POL_	0x31
#define	_PACKET_PREAMBLE_	0xFF

#define	TIMEOUT_ERR	0x5A

struct TPacket 
{
	uint8_t preamble;	// преамбула = 0xFF
	uint8_t cnt;		// счетчик пакетов
	uint8_t type;		// тип пакета
	uint8_t length;		// длина поля данных
	uint8_t data[255 + 1];	// массив данных, плюс место под CRC8
}; 
static const u8 _TPacket_HEADER_LEN_ = sizeof(TPacket) - sizeof(TPacket::data);

void PacketParser(u8 byte, u8 error = 0);
u8 calcCRC(u8 *data, u16 len, u8 polinom);
u32 sendPacket(u8 *data, u16 len, u16 counter);
u32 breakRecieve();

//------------------------------------------------------------------------------
u32 USART1_Read()
{
	while(1)
	{
		//	check to ovr
		if(USART1->SR && USART_SR_ORE)
		{
			while(USART1->SR && USART_SR_ORE)
				USART1->DR;
			PacketParser(0,USART_SR_ORE);
			return 0;
		}
		//	if RDR not empty, go to execution
		else if(USART1->SR && USART_SR_RXNE)
			PacketParser(USART1->DR);
		//	if RDR is empty, enabled interrupt and wait next byte
		else
			USART1->CR1 &= USART_CR1_RXNEIE;
		return 0;
	}
}
//------------------------------------------------------------------------------
u32 breakRecieve()
{
	PacketParser(0, TIMEOUT_ERR);
	return 0;
}

//------------------------------------------------------------------------------
void PacketParser(u8 byte, u8 error)
{
	static u8 isInRxProcess = _FALSE_;
	static u16 byteCounter = 0;
	static u8* packet = 0;
	
	//	execution ovr or timeout error
	if((error == USART_SR_ORE) || (error == TIMEOUT_ERR))
	{
		isInRxProcess = _FALSE_;
		byteCounter = 0;
		delete[] packet;
		return;
	}

	//	search preamble
	if(!isInRxProcess && byte == _PACKET_PREAMBLE_)
	{
		isInRxProcess = _TRUE_;
		packet = new u8[sizeof(TPacket)];
		packet[byteCounter++] = _PACKET_PREAMBLE_;
		//	timeout for abort recieve
		System::set_time_out(10 msec, (pulfv) breakRecieve);
	}
	//	read bytes
	else
	{
		packet[byteCounter++] = byte;
		//	check lenght
		if((byteCounter > _TPacket_HEADER_LEN_) && (byteCounter == (((TPacket*)packet)->length + 1)))
		{
			if(packet[byteCounter - 1] == calcCRC(packet, ((TPacket*)packet)->length + _TPacket_HEADER_LEN_, _CRC_POL_))
			{
				//	update crc
				((TPacket*)packet)->length += 0x80;
				packet[byteCounter - 1] = calcCRC(packet, ((TPacket*)packet)->length + _TPacket_HEADER_LEN_, _CRC_POL_);
				//	send
				System::push_queue((pulf3ul)sendPacket, (u32)packet, ((TPacket*)packet)->length + 1 + _TPacket_HEADER_LEN_, 0);	
				//	reset states for new recieve
				System::clr_time_out((pulf3ul)breakRecieve);
				isInRxProcess = _FALSE_;
				byteCounter = 0;
			}
		}
	}
}

//------------------------------------------------------------------------------
u32 sendPacket(u8 *data, u16 len, u16 counter = 0)
{
	static u8 isBusy = _FALSE_;
	
	//	if a new packet, when old sending
	if(isBusy && (counter == 0))
	{
		System::set_time_out(1 msec, (pulf3ul)sendPacket,(u32)data, (u32)len, 0);
		return 0;
	}
	//	it's new packet
	else
		isBusy = _TRUE_;
	
	//	tx
	if(USART1->SR & USART_SR_TXE)
	{
		USART1->DR = data[counter++];
		//	if all data transferred
		if(counter == len)
		{
			isBusy = _FALSE_;
			delete[] data;
		}
		else
			System::push_queue((pulf3ul)sendPacket, (u32)data, len, counter);
	}
	return 0;
}

//------------------------------------------------------------------------------
u8 calcCRC(u8 *data, u16 len, u8 polinom)
{
    u8 crc = 0xff;
    u16 i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (u8)((crc << 1) ^ polinom);
            else
                crc <<= 1;
        }
    }
    return crc;
}

//------------------------------------------------------------------------------
extern "C" void USART1_IRQHandler(void)
{
	USART1->CR1 &= USART_CR1_RXNEIE;
	System::push_queue((pulfv)USART1_Read);
}


//------------------------------------------------------------------------------
//														Timer
//------------------------------------------------------------------------------
#define UPDATE_CC1	(TIM2->CCR1 + 1)
#define UPDATE_CC2	(TIM2->CCR1 + 1000)
#define UPDATE_CC3	(TIM2->CCR1 + 100)

struct TPort
{
	u8 b0		:1;
	u8 b1		:1;
	u8 b2		:1;
	u8 b3		:1;
	u8 b4		:1;
	u8 b5		:1;
	u8 b6		:1;
	u8 b7		:1;
	u8 b8		:1;
	u8 b9		:1;
	u8 b10	:1;
	u8 b11	:1;
	u8 b12	:1;
	u8 b13	:1;
	u8 b14	:1;
	u8 b15	:1;
};

static volatile TPort* PortB = (TPort*)(&GPIOB->ODR);

extern "C" void TIM2_IRQHandler(void) 
{
	u16 statusReg = TIM2->SR;
	TIM2->SR = 0;
	//------
	//	10 kHz bit toggle 
	if(statusReg & TIM_SR_CC1IF)
	{
		//	update for next action
		TIM2->CCR1 += 1;
		PortB->b0 ^= 1;
	}
	
	//------
	//	1 kHz bit toggle 
	if(statusReg & TIM_SR_CC3IF)
	{
		//	update for next action
		TIM2->CCR3 += 100;	
		PortB->b1 ^= 1;
	}
	
	//------
	//	100 Hz adc start 
	if(statusReg & TIM_SR_CC2IF)
		//	update for next action
		TIM2->CCR2 += 1000;
		
	//------
	//	300 Hz bit toggle
	if(statusReg & TIM_SR_UIF)
	{
		// reset CCR to start values
		TIM2->CCR1 = 1;
		TIM2->CCR3 = 100;
		TIM2->CCR2 = 1000;
		
		PortB->b2 ^= 1;
	}
}


//------------------------------------------------------------------------------
//														ADC
//------------------------------------------------------------------------------
#define		NUM_SAMPLES		32
u32 average(u32 sum)
{
	u32 res = sum / NUM_SAMPLES;
	return 0;
}
extern "C" void ADC1_2_IRQHandler(void)
{
	static u32 sum = 0, counter = 0;
	
	sum += ADC1->DR & 0x0000FFFF;
	if(counter++ == NUM_SAMPLES)
	{
		sum = 0;
		counter = 0;
		System::push_queue((pulf1ul)average, sum);
	}
	ADC1->SR=0;
}





