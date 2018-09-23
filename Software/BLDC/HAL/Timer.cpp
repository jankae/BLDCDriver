#include "Timer.hpp"

#include "stm32f3xx_hal.h"
#include "Logging.hpp"

static HAL::BLDC::Timer::Callback cb;

static bool initialized = false;

static constexpr uint8_t busClockMHz = 64;

static void Init() {
    __HAL_RCC_TIM7_CLK_ENABLE();
    // timer operates in one shot mode
//    TIM7->CR1 = TIM_CR1_OPM;
    HAL_NVIC_SetPriority(TIM7_DAC2_IRQn, 6 ,0);
    HAL_NVIC_EnableIRQ(TIM7_DAC2_IRQn);
	Log::Uart(Log::Lvl::Dbg, "Initialized one shot timer");
}

void HAL::BLDC::Timer::Schedule(uint32_t usTillExecution, Callback ptr) {
	Log::Uart(Log::Lvl::Dbg, "Set cb for %luus (Tick: %lu, CB: %p)", usTillExecution, HAL_GetTick(), ptr);

	if(!initialized) {
		Init();
		initialized = true;
	}
	if(cb) {
		// abort last callback (also stops timer)
		Abort();
	}

	// calculate prescaler
	uint16_t psc = busClockMHz;
	if (usTillExecution >= 65536) {
		// needs a higher prescaler to reach requested time
		psc = usTillExecution * busClockMHz / 65536 + 1;
	}
	const uint16_t arr = usTillExecution * busClockMHz / psc;

	TIM7->PSC = psc - 1;
	TIM7->ARR = arr - 1;
	TIM7->CNT = 0;

	cb = ptr;

	// update timer registers
	TIM7->EGR = TIM_EGR_UG;
	// clear potential pending interrupt flag
	TIM7->SR &= ~TIM_SR_UIF;
	// enable interrupt and start timer
	TIM7->DIER = TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
}

void HAL::BLDC::Timer::Abort(void) {
	// stop timer and disable interrupt
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->DIER &= ~TIM_DIER_UIE;
	cb = nullptr;
}

extern "C" {
void TIM7_DAC2_IRQHandler(void) {
	if (TIM7->SR & TIM_SR_UIF) {
		// clear interrupt flag
		TIM7->SR &= ~TIM_SR_UIF;
		TIM7->CR1 &= ~TIM_CR1_CEN;
		if (cb) {
			Log::Uart(Log::Lvl::Dbg, "Exec cb (Tick: %lu CB: %p)", HAL_GetTick(), cb);
			auto buf = cb;
			cb = nullptr;
			buf();
			Log::Uart(Log::Lvl::Dbg, "Exit cb");
		}
	}
}
}
