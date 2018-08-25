#include "Tests.hpp"

#include "stm32f3xx_hal.h"
#include "lowlevel.hpp"
#include "Logging.hpp"

using namespace HAL::BLDC;

void Test::SetMidPWM(void) {
	Log::Uart(Log::Lvl::Inf, "Test, setting mid PWM");
	LowLevel::SetPWM(LowLevel::MaxPWM/2);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
}
