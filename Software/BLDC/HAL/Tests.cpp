#include "Tests.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"
#include "lowlevel.hpp"
#include "Logging.hpp"
#include "Driver.hpp"
#include "InductanceSensing.hpp"
#include "PowerADC.hpp"
#include "Persistance.hpp"

using namespace HAL::BLDC;

void Test::DifferentPWMs(void) {
	Log::Uart(Log::Lvl::Inf, "Test, setting different PWMs");
	LowLevel::SetPWM(100);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
}

void Test::MotorFunctions(void) {
	auto Start = [](Driver &d) -> bool {
		constexpr uint8_t attempts = 10;
		uint8_t attempt_cnt = 0;
		while(!d.IsRunning()) {
			attempt_cnt++;
			if(attempt_cnt > attempts) {
				Log::Uart(Log::Lvl::Err, "Unable to start motor");
				return false;
			}
			Log::Uart(Log::Lvl::Dbg, "Starting motor: attempt %d", attempt_cnt);
			d.InitiateStart();
			vTaskDelay(100);
		}
		return true;
	};

	Driver d;
	switch(d.Test()) {
	case Driver::TestResult::Failure:
		Log::Uart(Log::Lvl::Crt, "Driver reports hardware failure");
		return;
	case Driver::TestResult::NoMotor:
		Log::Uart(Log::Lvl::Err, "Driver reports no motor");
		return;
	}
	Log::Uart(Log::Lvl::Inf, "Driver test passed");

	{
		/* Speed change Test */
		constexpr uint16_t maxTestPWM = 500;
		constexpr uint16_t minTestPWM = 50;
		constexpr uint16_t minPWMStep = 30;
		constexpr uint16_t maxPWMStep = 450;
		constexpr uint16_t stepIncr = 30;

		HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
		if(!Start(d)) {
			return;
		}

		vTaskDelay(1000);
		d.SetPWM(300);
		vTaskDelay(500);
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
		d.SetPWM(100);
		vTaskDelay(5000);
//		d.FreeRunning();

//		for (uint16_t i = minPWMStep; i <= maxPWMStep; i += stepIncr) {
//			Log::Uart(Log::Lvl::Dbg, "Speed change, step: %d", i);
//			int16_t j;
//			for (j = minTestPWM; j <= maxTestPWM; j += i) {
//				Log::Uart(Log::Lvl::Dbg, "PWM: %d", j);
//				d.SetPWM(j);
//				vTaskDelay(300);
//				if (!d.IsRunning()) {
//					d.FreeRunning();
//					Log::Uart(Log::Lvl::Wrn,
//							"Failed speed change test: PWM: %d, step: %d",
//							j, i);
//					return;
//				}
//			}
//			j -= i;
//			for (;j >= minTestPWM; j -= i) {
//				Log::Uart(Log::Lvl::Dbg, "PWM: %d", j);
//				d.SetPWM(j);
//				vTaskDelay(300);
//				if (!d.IsRunning()) {
//					d.FreeRunning();
//					Log::Uart(Log::Lvl::Wrn,
//							"Failed speed change test: PWM: %d, step: -%d",
//							j, i);
//					return;
//				}
//			}
//		}
//		d.FreeRunning();
//
//		Log::Uart(Log::Lvl::Inf, "Speed change test passed");

	}

//	while (1) {
//		d.SetDirection(Driver::Direction::Forward);
//		d.InitiateStart();
//		vTaskDelay(2000);
//		for (uint16_t i = 100; i < 500; i++) {
//			d.SetPWM(i);
//			vTaskDelay(10);
//		}
//		for (uint16_t i = 500; i > 50; i--) {
//			d.SetPWM(i);
//			vTaskDelay(10);
//		}
////		d.FreeRunning();
//		vTaskDelay(1000);
//		d.SetDirection(Driver::Direction::Reverse);
////		d.InitiateStart();
//		vTaskDelay(2000);
//		d.Stop();
//		vTaskDelay(3000);
//	}
}

static void SetStep(uint8_t step) {
	switch (step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		break;
	}
}

void Test::InductanceSense() {
	while (1) {
		uint16_t pos = InductanceSensing::RotorPosition();
		Log::Uart(Log::Lvl::Inf, "Pos: %d", pos);
		pos = (9 - pos) % 6;
		LowLevel::SetPWM(100);
		SetStep(pos);
		vTaskDelay(100);
		LowLevel::SetPWM(0);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		vTaskDelay(1000);
	}
}

void Test::MotorManualStart(void) {
	Driver d;
	while (1) {
		Log::Uart(Log::Lvl::Inf, "Waiting for external start");
		while(!d.GotValidPosition()) {
			vTaskDelay(10);
		}
		d.InitiateStart();
		vTaskDelay(2000);
		d.FreeRunning();
		while(d.GotValidPosition()) {
			vTaskDelay(10);
		}
		vTaskDelay(500);
	}
}

void Test::PowerADC() {
	Driver d;
	d.InitiateStart();
	while(1) {
		auto m = HAL::BLDC::PowerADC::Get();
		Log::Uart(Log::Lvl::Inf, "U: %lu, I: %ld", m.voltage, m.current);
		vTaskDelay(5);
	}
}

void Test::ManualCommutation() {
	uint8_t step = 0;

	for (uint8_t i = 0; i < 12; i++) {
		step = (step + 1) % 6;

		LowLevel::SetPWM(100);
		switch (step) {
		case 0:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
			break;
		case 1:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
			break;
		case 2:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			break;
		case 3:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
			break;
		case 4:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
			break;
		case 5:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			break;
		}
		vTaskDelay(2000);
	}
	LowLevel::SetPWM(0);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
}

//static uint32_t testdata[4] __attribute__ ((section (".ccmpersist")));
//
//void Test::PersistenceTest() {
//	Persistance::Load();
//	Log::Uart(Log::Lvl::Inf, "Previous data: %x %x %x %x", testdata[0], testdata[1], testdata[2], testdata[3]);
//	testdata[0] = 0xdeadbeef;
//	testdata[1] = 0xdeadbeef;
//	testdata[2] = 0xdeadbeef;
//	testdata[3] = 0xdeadbeef;
//	Log::Uart(Log::Lvl::Inf, "Set data: %x %x %x %x", testdata[0], testdata[1], testdata[2], testdata[3]);
//	Persistance::Store();
//	Persistance::Load();
//	Log::Uart(Log::Lvl::Inf, "Afterwards data: %x %x %x %x", testdata[0], testdata[1], testdata[2], testdata[3]);
//}
