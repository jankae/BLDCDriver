#include "Startup.hpp"

#include "Logging.hpp"
#include "lowlevel.hpp"
#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "PowerADC.hpp"
#include "Persistance.hpp"

#include "Tests.hpp"

void Start() {
	Persistance::Load();

	Log::Init(Log::Lvl::Inf);

	HAL::BLDC::PowerADC::Init();
	HAL::BLDC::LowLevel::Init();

	vTaskDelay(100);
	Log::Uart(Log::Lvl::Inf, "Start");

	vTaskDelay(500);

//	Test::PersistenceTest();
	Test::PowerADC();
//	Test::ManualCommutation();
//	Test::InductanceSense();
//	Test::MotorFunctions();
//	Test::MotorManualStart();
//	Test::DifferentPWMs();
	while(1);
}
