#include "Startup.h"

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
#include "i2c_slave.hpp"
#include "Motor.hpp"
#include "cast.hpp"

#include "Tests.hpp"

using namespace HAL::BLDC;

Core::Sysinfo sys;
Core::Propeller::Data propdata __attribute__ ((section (".ccmpersist")));
Driver::Data motdata __attribute__ ((section (".ccmpersist")));
I2CSlave *i2cSlave;
uint8_t i2cDummyData[10];

void Start() {
	Persistance::Load();

	// Initialize hardware
	Log::Init(Log::Lvl::Dbg);

	vTaskDelay(10);
	Log::Uart(Log::Lvl::Inf, "Start");

	HAL::BLDC::PowerADC::Init();
	HAL::BLDC::LowLevel::Init();
	auto driver = new HAL::BLDC::Driver(&motdata);
	sys.driver = driver;

	// perform low level hardware check of driver
	switch(sys.driver->Test()) {
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

	Log::Uart(Log::Lvl::Inf, "Starting motor controller");
	auto mot = new Core::Motor(sys);

	i2cSlave = new I2CSlave(I2C1, 0x50);
	i2cSlave->SetReadBase(&mot->outState, sizeof(Core::Motor::OutState));
	i2cSlave->SetWriteBase(&mot->inState, sizeof(Core::Motor::InState));
	i2cSlave->SetCallback(
			pmf_cast<void (*)(void*), Core::Motor, &Core::Motor::NewData>::cfn,
			mot);
	Log::Uart(Log::Lvl::Inf, "I2C slave initialized");

//	driver->Calibrate();
//	Persistance::Store();
//	Test::MotorFunctions();
//	Test::MotorCharacterisation();
//	Test::WindEstimation();

//	Test::PersistenceTest();
//	Test::PowerADC();
//	Test::ManualCommutation();
//	Test::InductanceSense();
//	Test::MotorFunctions();
//	Test::MotorManualStart();
//	Test::DifferentPWMs();

	// Startup completed, this task is no longer needed
	vTaskDelete(nullptr);
}

void I2CInterrupt() {
	i2cSlave->EventInterrupt();
}
