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

using namespace HAL::BLDC;

Core::Sysinfo sys;
Core::Propeller::Data propdata __attribute__ ((section (".ccmpersist")));

Driver *d;

void Start() {
	Persistance::Load();

	// Initialize hardware
	Log::Init(Log::Lvl::Inf);

	vTaskDelay(10);
	Log::Uart(Log::Lvl::Inf, "Start");

	HAL::BLDC::PowerADC::Init();
	HAL::BLDC::LowLevel::Init();
	d = new HAL::BLDC::Driver();

	// perform low level hardware check of driver
	switch(d->Test()) {
	case Driver::TestResult::OK:
		Log::Uart(Log::Lvl::Inf, "Driver passed test");
		break;
	case Driver::TestResult::NoMotor:
		Log::Uart(Log::Lvl::Err, "No motor detected");
		break;
	case Driver::TestResult::Failure:
		Log::Uart(Log::Lvl::Crt, "Hardware failure detected, shutting down");
		vTaskDelete(nullptr);
		return;
		break;
	}

	// create core objects
	sys.communication = new Core::Communication();
	sys.prop = new Core::Propeller(&propdata);

	// inform objects of each other
	sys.communication->SetSystemInfo(&sys);

	Test::MotorCharacterisation();

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
