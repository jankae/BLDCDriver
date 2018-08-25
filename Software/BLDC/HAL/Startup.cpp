#include "Startup.hpp"

#include "Logging.hpp"
#include "stm32f3xx_hal.h"

void Start() {
	HAL_Delay(100);
	Log::Uart(Log::Class::BLDC, Log::Lvl::Inf, "Start");
}
