#include "Logging.hpp"

#include <array>
#include <stdarg.h>
#include "Sysinfo.hpp"
#include "Communication.hpp"

std::array<enum Log::Lvl, (int) Log::Class::MAX> levels;

#include <string.h>
//#include "fifo.hpp"
#include "usart.h"

//static Fifo<uint8_t, 1024> fifo __attribute__ ((section (".ccmram")));
static constexpr uint16_t LogBufSize = 512;
static uint8_t buf1[LogBufSize];
static uint8_t buf2[LogBufSize];
static uint8_t *buffering;
static uint8_t *transmitting;
static volatile uint16_t bufIndex;
static volatile bool transmissionActive;

#define USART_HANDLE 		huart3
extern UART_HandleTypeDef 	USART_HANDLE;

#define USART				3
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

static void init() {
	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(NVIC_ISR, 1, 0);
	HAL_NVIC_EnableIRQ(NVIC_ISR);

	buffering = buf1;
	transmitting = buf2;
	bufIndex = 0;
	transmissionActive = false;
}

__weak void LogRedirect(const char *data, uint16_t length){
	UNUSED(data);
	UNUSED(length);
}

static void NextTransmission() {
	transmissionActive = true;
	uint16_t transmissionSize = bufIndex;
	HAL_UART_Transmit_DMA(&USART_HANDLE, buffering, transmissionSize);
	{
		CriticalSection crit;
		uint8_t *b = buffering;
		buffering = transmitting;
		transmitting = b;
		bufIndex -= transmissionSize;
		if (bufIndex) {
			memcpy(buffering, &transmitting[transmissionSize], bufIndex);
		}
	}
}

void write(const char *start, const char *end) {
	LogRedirect(start, end - start);
	CriticalSection crit;
	if (end - start < LogBufSize - bufIndex) {
		memcpy(&buffering[bufIndex], start, end - start);
		bufIndex += (end - start);
		if (!transmissionActive) {
			// enable ISR
			NVIC_SetPendingIRQ(NVIC_ISR);
		}
	}
}

extern "C" {
/* Implemented directly here for speed reasons. Disable interrupt in CubeMX! */
void HANDLER(void)
{
//	if(USART_BASE->ISR & USART_ISR_TC) {
		// clear flag
//		USART_BASE->ICR = USART_ICR_TCCF;
		// This interrupt is called while the UART is still in transmit mode
		// despite having finished transmitting. To allow a new transmission
		// to start, abort the already finished transmission, thereby resetting
		// the UART state in the ST HAL
		if (transmissionActive) {
			HAL_UART_Abort(&USART_HANDLE);
		}
		if (bufIndex > 0) {
			NextTransmission();
		} else {
			transmissionActive = false;
		}
		// disable ISR
		USART_BASE->CR1 &= ~USART_CR1_TCIE;
//	}
}
}


void Log::Init(enum Lvl lvl) {
	for (auto i = 0; i < (int) Log::Class::MAX; i++) {
		levels[i] = lvl;
	}
	init();
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
		char time[6];
		uint32_t tick = HAL_GetTick();
		time[0] = (tick/10000)%10 + '0';
		time[1] = (tick/1000)%10 + '0';
		time[2] = (tick/100)%10 + '0';
		time[3] = (tick/10)%10 + '0';
		time[4] = (tick/1)%10 + '0';
		time[5] = ':';
		write(time, time + 6);
		switch(lvl) {
		case Lvl::Dbg:
			WriteString("[DBG]:");
			break;
		case Lvl::Inf:
			WriteString("[INF]:");
			break;
		case Lvl::Wrn:
			WriteString("[WRN]:");
			break;
		case Lvl::Err:
			WriteString("[ERR]:");
			break;
		case Lvl::Crt:
			WriteString("[CRT]:");
			break;
		}
		char buffer[128];
		va_list arp;
		va_start(arp, fmt);
		int len = vsnprintf(buffer, sizeof(buffer), fmt, arp);
		va_end(arp);

		write(buffer, buffer + len);
		WriteString("\n");
	}
}

void Log::WriteString(const char* s) {
	write(s, s + strlen(s));
}
