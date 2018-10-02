#include "Startup.hpp"

#include "Logging.hpp"
#include "lowlevel.hpp"
#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "PowerADC.hpp"
#include "Persistance.hpp"
#include "Sysinfo.hpp"
#include "Communication.hpp"
#include "Driver.hpp"
#include "Propeller.hpp"

#include "Tests.hpp"

Core::Sysinfo sys;
Core::Propeller::Data propdata __attribute__ ((section (".ccmpersist")));

void Start() {
	Persistance::Load();

	// Initialize hardware
	Log::Init(Log::Lvl::Dbg);

	vTaskDelay(10);
	Log::Uart(Log::Lvl::Inf, "Start");

	HAL::BLDC::PowerADC::Init();
	HAL::BLDC::LowLevel::Init();
	auto driver = new HAL::BLDC::Driver();

	// create driver and core objects
	sys.communication = new Core::Communication();
	sys.prop = new Core::Propeller(&propdata);

	// inform objects of each other
	sys.communication->SetSystemInfo(&sys);

	// Startup completed, this task is no longer needed
	vTaskDelete(nullptr);

//	Test::PersistenceTest();
//	Test::PowerADC();
//	Test::ManualCommutation();
//	Test::InductanceSense();
//	Test::MotorFunctions();
//	Test::MotorManualStart();
//	Test::DifferentPWMs();
}
