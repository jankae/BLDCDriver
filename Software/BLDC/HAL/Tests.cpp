#include "Tests.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"
#include "lowlevel.hpp"
#include "Logging.hpp"
#include "Driver.hpp"
#include "InductanceSensing.hpp"
#include "PowerADC.hpp"

using namespace HAL::BLDC;

void Test::SetMidPWM(void) {
	Log::Uart(Log::Lvl::Inf, "Test, setting mid PWM");
	LowLevel::SetPWM(100);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
}

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

//	{
//		/* Repower during idle Test */
//		for(uint16_t i = 100;i>=10;i-= 10) {
//			if(!Start(d)) {
//				return;
//			}
//			vTaskDelay(200);
//			d.SetPWM(200);
//			vTaskDelay(300);
//			d.FreeRunning();
//			vTaskDelay(i);
//			d.InitiateStart();
//			vTaskDelay(1000);
//			if(d.IsRunning()) {
//				Log::Uart(Log::Lvl::Inf, "Repowering after %dms worked", i);
//			} else {
//				Log::Uart(Log::Lvl::Inf, "Repowering after %dms failed", i);
//			}
//			d.FreeRunning();
//			vTaskDelay(1000);
//			d.Stop();
//			vTaskDelay(1000);
//		}
//	}
//	return;

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
		d.SetPWM(250);
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
//	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
//	HAL_GPIO_TogglePin(TRIGGER_GPIO_Port, TRIGGER_Pin);
	switch (step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
//		Detector::SetPhase(Detector::Phase::B, true);
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
//		Detector::SetPhase(Detector::Phase::A, false);
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
//		Detector::SetPhase(Detector::Phase::C, true);
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
//		Detector::SetPhase(Detector::Phase::B, false);
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
//		Detector::SetPhase(Detector::Phase::A, true);
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
//		Detector::SetPhase(Detector::Phase::C, false);
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
	vTaskDelay(1000);
	HAL::BLDC::PowerADC::Pause();
	PowerADC::PrintBuffer();
	PowerADC::Resume();
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
//			Detector::SetPhase(Detector::Phase::B, false);
			break;
		case 1:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
//			Detector::SetPhase(Detector::Phase::A, true);
			break;
		case 2:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
//			Detector::SetPhase(Detector::Phase::C, false);
			break;
		case 3:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
//			Detector::SetPhase(Detector::Phase::B, true);
			break;
		case 4:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
//			Detector::SetPhase(Detector::Phase::A, false);
			break;
		case 5:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
//			Detector::SetPhase(Detector::Phase::C, true);
			break;
		}
		vTaskDelay(2000);
		uint16_t sample[3];
//		sample[0] = Detector::GetLastSample(Detector::Phase::A);
//		sample[1] = Detector::GetLastSample(Detector::Phase::B);
//		sample[2] = Detector::GetLastSample(Detector::Phase::C);
		LowLevel::SetPWM(0);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		Log::Uart(Log::Lvl::Inf, "Phases: %d;%d;%d", sample[0], sample[1], sample[2]);
//		LowLevel::SetPWM(0);
//		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
//		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
//		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
//
//		uint8_t sector = InductanceSensing::RotorPosition();
//		Log::Uart(Log::Lvl::Inf, "Step: %d, Sector: %d", step, sector);
	}
	LowLevel::SetPWM(0);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
}
