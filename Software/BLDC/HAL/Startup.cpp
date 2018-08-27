#include "Startup.hpp"

#include "Logging.hpp"
#include "lowlevel.hpp"
#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Detector.hpp"

#include "Tests.hpp"

extern uint32_t timeUS;

void Start() {
	Log::Init(Log::Lvl::Inf);

	HAL::BLDC::Detector::Init();
	HAL::BLDC::LowLevel::Init();

	vTaskDelay(100);
	Log::Uart(Log::Lvl::Inf, "Start");

	vTaskDelay(2000);
	Test::MotorStart();
//	Test::TimerTest();
//	Test::SetMidPWM();
//	Test::DifferentPWMs();
//	HAL::BLDC::Detector::Enable(HAL::BLDC::Detector::Phase::C, nullptr);
	while(1);
}
