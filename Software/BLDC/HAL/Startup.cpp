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

	i2cSlave = new I2CSlave(I2C1, 0x40);
	i2cSlave->SetReadBase(i2cDummyData, sizeof(i2cDummyData));
	i2cSlave->SetWriteBase(i2cDummyData, sizeof(i2cDummyData));
	i2cSlave->SetCallback([](void *){
		Log::Uart(Log::Lvl::Dbg, "I2C Callback");
	}, nullptr);
	Log::Uart(Log::Lvl::Dbg, "I2C slave initialized");

	HAL::BLDC::PowerADC::Init();
	HAL::BLDC::LowLevel::Init();
	sys.driver = new HAL::BLDC::Driver(&motdata);

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

//	sys.driver->Calibrate();
//	Persistance::Store();
//	Test::MotorFunctions();
//	Test::MotorCharacterisation();
//	Test::WindEstimation();

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

void I2CInterrupt() {
	i2cSlave->EventInterrupt();
}
