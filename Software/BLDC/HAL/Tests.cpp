#include "Tests.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"
#include "lowlevel.hpp"
#include "Logging.hpp"
#include "Timer.hpp"
#include "Driver.hpp"
#include "Detector.hpp"
#include "InductanceSensing.hpp"

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

void Test::MotorStart(void) {
	Log::Uart(Log::Lvl::Inf, "Test, attempting to start motor");
	Driver d;
	while (1) {
		d.InitiateStart();
		vTaskDelay(2000);
		d.FreeRunning();
		vTaskDelay(1200);
		d.InitiateStart();
		vTaskDelay(2000);
		d.FreeRunning();
		vTaskDelay(3000);
	}
}

void Test::TimerTest(void) {
	Log::Uart(Log::Lvl::Inf, "Test, registering callback in 1s (now: %lu)", HAL_GetTick());
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
	Timer::Schedule(1000000, [](){
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
		Log::Uart(Log::Lvl::Inf, "Callback executed at %lu", HAL_GetTick());
		Log::Uart(Log::Lvl::Inf, "Test, registering callback in 20ms (now: %lu)", HAL_GetTick());
		Timer::Schedule(20000, [](){
			HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
			Log::Uart(Log::Lvl::Inf, "Callback executed at %lu", HAL_GetTick());
		});
	});
}

static void SetStep(uint8_t step) {
//	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
//	HAL_GPIO_TogglePin(TRIGGER_GPIO_Port, TRIGGER_Pin);
	switch (step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		Detector::SetPhase(Detector::Phase::B, true);
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		Detector::SetPhase(Detector::Phase::A, false);
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		Detector::SetPhase(Detector::Phase::C, true);
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		Detector::SetPhase(Detector::Phase::B, false);
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		Detector::SetPhase(Detector::Phase::A, true);
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		Detector::SetPhase(Detector::Phase::C, false);
		break;
	}
}

void Test::InductanceSense() {
	while (1) {
		uint16_t pos = InductanceSensing::RotorPosition();
		pos = (8 - pos) % 6;
		LowLevel::SetPWM(100);
		SetStep(pos);
		Detector::Enable(nullptr);
		while(Detector::isEnabled());
		Detector::PrintBuffer();
//		Log::Uart(Log::Lvl::Inf, "Pos: %d", pos);
		vTaskDelay(1000);
	}
}

void Test::MotorManualStart(void) {
	Driver d;
	while (1) {
		Log::Uart(Log::Lvl::Inf, "Waiting for external start");
		while(d.GetState() == Driver::State::Stopped) {
			vTaskDelay(10);
		}
		d.InitiateStart();
		vTaskDelay(2000);
		d.FreeRunning();
		while(d.GetState() != Driver::State::Stopped) {
			vTaskDelay(10);
		}
		vTaskDelay(500);
	}
}

void Test::ManualCommutation() {
	uint8_t step = 0;

	for (uint8_t i = 0; i < 42; i++) {
		step = (step + 1) % 6;

		LowLevel::SetPWM(50);
		switch (step) {
		case 0:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
			Detector::SetPhase(Detector::Phase::B, false);
			break;
		case 1:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
			Detector::SetPhase(Detector::Phase::A, true);
			break;
		case 2:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			Detector::SetPhase(Detector::Phase::C, false);
			break;
		case 3:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
			Detector::SetPhase(Detector::Phase::B, true);
			break;
		case 4:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
			Detector::SetPhase(Detector::Phase::A, false);
			break;
		case 5:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			Detector::SetPhase(Detector::Phase::C, true);
			break;
		}
		vTaskDelay(1000);
		LowLevel::SetPWM(0);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);

		uint8_t sector = InductanceSensing::RotorPosition();
		Log::Uart(Log::Lvl::Inf, "Step: %d, Sector: %d", step, sector);
	}
}
