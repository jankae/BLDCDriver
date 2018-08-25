#include "Startup.hpp"

#include "Logging.hpp"
#include "lowlevel.hpp"
#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Detector.hpp"

#include "Tests.hpp"

void dummy(uint32_t usSinceLast, uint32_t timeSinceCrossing) {

}

extern uint32_t DMAcnt;

void Start() {
	HAL::BLDC::Detector::Init(dummy);
	HAL::BLDC::LowLevel::Init();

	uint32_t start = DMAcnt;
	vTaskDelay(100);
	uint32_t stop = DMAcnt;
	uint32_t diff = stop - start;
	uint32_t sampleRate = diff * 252 / 10;
//	Log::Uart(Log::Lvl::Inf, "DMA Samples: %d", diff);

	Log::Uart(Log::Lvl::Inf, "Start: %d", stop);

	vTaskDelay(2000);
	Test::SetMidPWM();
}
