#include "Tests.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"
#include "lowlevel.hpp"
#include "Logging.hpp"
#include "Timer.hpp"
#include "Driver.hpp"
#include "Detector.hpp"

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
	d.InitiateStart();
//	vTaskDelay(1000);
//	for(uint16_t pwm = 200;pwm>0;pwm--) {
//		d.SetPWM(pwm);
//		Log::Uart(Log::Lvl::Inf, "PWM: %d", pwm);
//		vTaskDelay(100);
//	}
//	while(1) {
//	d.InitiateStart();
//	vTaskDelay(2000);
//	d.FreeRunning();
//	vTaskDelay(3000);
//	d.InitiateStart();
//	vTaskDelay(2000);
//	d.Stop();
//	vTaskDelay(3000);
//	}
	vTaskDelay(2000);
	d.SetPWM(600);
	constexpr uint16_t maxRPM = 600;
//	for(uint16_t pwm = 100;pwm<maxRPM;pwm++) {
//		d.SetPWM(pwm);
//		vTaskDelay(1);
//	}
	vTaskDelay(1000);
	for(uint16_t pwm = maxRPM;pwm>100;pwm--) {
		d.SetPWM(pwm);
		vTaskDelay(1);
	}
//	for (uint16_t pwm = 100; pwm < maxRPM; pwm++) {
//		d.SetPWM(pwm);
//		vTaskDelay(5);
//	}
	d.SetPWM(100);
//	d.SetPWM(50);
//	for(uint16_t pwm = 500;pwm>100;pwm--) {
//		d.SetPWM(pwm);
//		vTaskDelay(2);
//	}
//	d.SetPWM(500);
//	vTaskDelay(2000);
//	d.SetPWM(100);
//	vTaskDelay(2000);
//	d.SetPWM(0);
//	Detector::PrintBuffer();
}

void Test::TimerTest(void) {
//	Log::Uart(Log::Lvl::Inf, "Test, registering callback in 10ms (now: %lu)", HAL_GetTick());
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
	Timer::Schedule(200, [](){
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
//		Log::Uart(Log::Lvl::Inf, "Callback executed at %lu", HAL_GetTick());
//		Log::Uart(Log::Lvl::Inf, "Test, registering callback in 20ms (now: %lu)", HAL_GetTick());
		Timer::Schedule(100, [](){
			HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
//			Log::Uart(Log::Lvl::Inf, "Callback executed at %lu", HAL_GetTick());
		});
	});
}
