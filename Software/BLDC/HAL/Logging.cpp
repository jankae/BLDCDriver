#include "Logging.hpp"

#include <array>
#include <stdarg.h>

std::array<enum Log::Lvl, (int) Log::Class::MAX> levels;

#include <string.h>
#include "fifo.hpp"
#include "usart.h"

static bool initialized = false;
static Fifo<uint8_t, 1024> fifo;// __attribute__ ((section (".ccmram")));

#define USART 				3

/* Automatically build register and function names based on USART selection */
#define USART_M2(y) 		USART ## y
#define USART_M1(y)  		USART_M2(y)
#define USART_BASE			USART_M1(USART)

#define HANDLER_M2(x) 		USART ## x ## _IRQHandler
#define HANDLER_M1(x)  		HANDLER_M2(x)
#define HANDLER				HANDLER_M1(USART)

#define NVIC_ISR_M2(x) 		USART ## x ## _IRQn
#define NVIC_ISR_M1(x)  	NVIC_ISR_M2(x)
#define NVIC_ISR			NVIC_ISR_M1(USART)

#define writeString(s) \
do { const char string[] = s; write(string, string + sizeof(string) - 1); } while(0)

static void init() {
	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(NVIC_ISR, 8, 0);
	HAL_NVIC_EnableIRQ(NVIC_ISR);

	fifo.clear();
	initialized = true;
}

__weak void LogRedirect(const char *data, uint16_t length){
	UNUSED(data);
	UNUSED(length);
}

void write(const char *start, const char *end) {
	LogRedirect(start, end - start);
	while(start != end) {
		fifo.enqueue(*start);
		start++;
	}
	USART_BASE->CR1 |= USART_CR1_TXEIE;
}

extern "C" {
/* Implemented directly here for speed reasons. Disable interrupt in CubeMX! */
void HANDLER(void)
{
	if (USART_BASE->ISR & USART_ISR_TXE) {
		uint8_t data = 0;
		fifo.dequeue(data);
		USART_BASE->TDR = data;
		if (fifo.getLevel() == 0) {
			/* complete buffer sent, disable interrupt */
			USART_BASE->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}
}


void Log::Init(enum Lvl lvl) {
	for (auto i = 0; i < (int) Log::Class::MAX; i++) {
		levels[i] = lvl;
	}
}

void Log::SetLevel(enum Class cls, enum Lvl lvl) {
	levels[(int) cls] = lvl;
}

void Log::Out(enum Class cls, enum Lvl lvl, const char* fmt, ...) {
	// only used in simulation
	UNUSED(cls);
	UNUSED(lvl);
	UNUSED(fmt);
}

void Log::Uart(enum Lvl lvl, const char* fmt, ...) {
	if((int) levels[(int) Class::BLDC] <= (int) lvl) {
		if(!initialized) {
			init();
		}
		switch(lvl) {
		case Lvl::Dbg:
			writeString("[DBG]:");
			break;
		case Lvl::Inf:
			writeString("[INF]:");
			break;
		case Lvl::Wrn:
			writeString("[WRN]:");
			break;
		case Lvl::Err:
			writeString("[ERR]:");
			break;
		case Lvl::Crt:
			writeString("[CRT]:");
			break;
		}
		char buffer[128];
		va_list arp;
		va_start(arp, fmt);
		int len = vsnprintf(buffer, sizeof(buffer), fmt, arp);
		va_end(arp);

		write(buffer, buffer + len);
		writeString("\n");
	}
}
