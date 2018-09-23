#include "Driver.hpp"

#include "lowlevel.hpp"
#include "Detector.hpp"
#include "Timer.hpp"
#include "Logging.hpp"

#include "stm32f3xx_hal.h"
#include "stm32f303x8.h"
#include "InductanceSensing.hpp"

using namespace HAL::BLDC;

static Driver::IncCallback IncCB;
static void* IncPtr;

static uint8_t CommutationStep;



static Driver::State state;
static uint32_t StartTime;
static uint16_t StartSteps;

static constexpr uint32_t StartSequenceLength = 140000; //sizeof(StartSequence)/sizeof(StartSequence[0]);
static constexpr uint32_t StartFinalRPM = 800;
static constexpr uint8_t MotorPoles = 12;
static constexpr uint32_t StartMaxPeriod = 10000;
static constexpr uint16_t StartFinalPWM = 101;
static constexpr uint16_t StartMinPWM = 100;

static uint32_t StartSequence(uint32_t time) {
	constexpr uint32_t FinalPeriod = 1000000UL
			/ (StartFinalRPM * 6 * MotorPoles / 2 / 60);
	constexpr uint64_t dividend = FinalPeriod * StartSequenceLength;
	uint64_t period = dividend / time;
	if (time == 0 || period > StartMaxPeriod)
		return StartMaxPeriod;
	else
		return period;
}

static uint16_t StartPWM(uint32_t time) {
	constexpr uint32_t divisor = StartSequenceLength / (StartFinalPWM - StartMinPWM);
	return StartMinPWM + time / divisor;
}

static uint32_t timeBetweenCommutations;

static void CrossingCallback(uint32_t usSinceLast, uint32_t timeSinceCrossing);
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

	if (state == Driver::State::Running) {
		uint32_t timeout = 20000;
		if(timeBetweenCommutations * 5 > timeout) {
			timeout = timeBetweenCommutations * 5;
		}
//		Timer::Schedule(timeout,
//				[]() {
//					Log::WriteChar('T');

//					Detector::Disable();
//					LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
//					LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
//					LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
//
//					state = Driver::State::Stopped;
//					Log::Uart(Log::Lvl::Err, "Timed out while waiting for commutation, motor stopped");
//					Timer::Schedule(100000, [](){
//
//					});
//		});
	}
}

static void NextStartStep() {
	Log::WriteChar('N');

//	Log::Uart(Log::Lvl::Dbg, "Start step %d", StartStep);
	CommutationStep = (CommutationStep + 1) % 6;
	Detector::Disable();
	SetStep(CommutationStep);
	if(state == Driver::State::Running) {
		// Successfully started, enter normal operation mode
		Detector::Enable(CrossingCallback);
		return;
	}
	// Schedule next start step
	auto length = StartSequence(StartTime);
	LowLevel::SetPWM(StartPWM(StartTime));
	StartTime += length;

	if(StartSteps >= 10) {
		Detector::Enable(CrossingCallback, 50);
	}
	StartSteps++;

	if (StartTime >= StartSequenceLength) {
		// TODO Start attempt failed
		Detector::Disable();
		Log::Uart(Log::Lvl::Err, "Failed to start motor");
		LowLevel::SetPWM(0);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		state = Driver::State::Stopped;
		return;
	}

	Timer::Schedule(length, NextStartStep);
}

static void CrossingCallback(uint32_t usSinceLast, uint32_t timeSinceCrossing) {
	Log::WriteChar('B');
	UNUSED(timeSinceCrossing);
//	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
	if (state == Driver::State::Starting) {
		state = Driver::State::Running;
		Log::Uart(Log::Lvl::Inf, "Motor started after %luus", StartTime);
		Detector::Disable();
		timeBetweenCommutations = StartSequence(StartTime);
//		return;
		// abort next scheduled start step
		Timer::Abort();
		LowLevel::SetPWM(100);
		// no previous commutation known, take a guess from the start sequence
		usSinceLast = StartSequence(StartTime);
//		timeSinceCrossing = StartSequence(StartTime) / 2;
	} else if (IncCB) {
		// motor is already running, report back crossing intervals to controller
		IncCB(IncPtr, usSinceLast);
	}

	// Disable detector until next commutation step
	Detector::Disable();
	CommutationStep = (CommutationStep + 1) % 6;
	// Calculate time until next 30° rotation
	uint32_t TimeToNextCommutation = usSinceLast / 2;// - timeSinceCrossing;
	Timer::Schedule(TimeToNextCommutation, []() {
		Log::WriteChar('M');
		SetStep(CommutationStep);
		Detector::Enable(CrossingCallback);
	});
//	Log::Uart(Log::Lvl::Inf, "next comm in %luus", TimeToNextCommutation);

	timeBetweenCommutations = usSinceLast;
}

static void IdleTrackingCB(uint8_t pos, bool valid) {
	CommutationStep = pos;
	if(!valid && state != Driver::State::Stopped) {
		// motor is running too slow for idle tracking, consider it stopped
		state = Driver::State::Stopped;
		Log::Uart(Log::Lvl::Inf, "...stopped");
	} else if(valid && state == Driver::State::Stopped) {
		state = Driver::State::Stopping;
		Log::Uart(Log::Lvl::Inf, "Motor started by external force");
	}
}

HAL::BLDC::Driver::Driver() {
	LowLevel::SetPWM(0);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);

	Detector::Disable();

	CommutationStep = 0;

	state = State::Stopped;

	Detector::EnableIdleTracking(IdleTrackingCB);
}

void HAL::BLDC::Driver::SetPWM(int16_t promille) {
	LowLevel::SetPWM(promille);
}

void HAL::BLDC::Driver::InitiateStart() {
	if (state == State::Stopped) {
		Log::Uart(Log::Lvl::Inf, "Initiating start sequence");
		state = State::Starting;
		uint8_t sector;
		do {
			sector = InductanceSensing::RotorPosition();
		} while (!sector);
		StartTime = 0;
		StartSteps = 0;
		if (sector == 0) {
			// unable to determine rotor position, use align and go
			LowLevel::SetPWM(30);
			Detector::Disable();
			SetStep(CommutationStep);
			Timer::Schedule(1000000, NextStartStep);
		} else {
			SetPWM(100);
			// rotor position determined, modify next commutation step accordingly
			CommutationStep = (7 - sector) % 6;
//		SetStep(CommutationStep);
//		Detector::Enable(CrossingCallback);
			NextStartStep();
		}
	} else if (state == State::Stopping){
		state = State::Running;
		timeBetweenCommutations = 100000;
		Log::Uart(Log::Lvl::Inf, "Repower idling motor");
		CommutationStep = (CommutationStep + 2) % 6;
		LowLevel::SetPWM(StartFinalPWM);
		SetStep(CommutationStep);
		Detector::DisableIdleTracking();
		Detector::Enable(CrossingCallback);
	}
}

void HAL::BLDC::Driver::RegisterIncCallback(IncCallback c, void* ptr) {
	IncCB = c;
	IncPtr = ptr;
}

Driver::State HAL::BLDC::Driver::GetState() {
	return state;
}

void HAL::BLDC::Driver::RegisterADCCallback(ADCCallback c, void* ptr) {
	UNUSED(c);
	UNUSED(ptr);
}

static void Idle() {
	Log::Uart(Log::Lvl::Inf, "Idle");
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
	LowLevel::SetPWM(0);
}

void HAL::BLDC::Driver::FreeRunning() {
	Timer::Abort();
	Detector::Disable();
	Idle();
	state = State::Stopping;
	Log::Uart(Log::Lvl::Inf, "Freerunning...");
	Detector::EnableIdleTracking(IdleTrackingCB);
}

void HAL::BLDC::Driver::Stop() {
	Log::Uart(Log::Lvl::Inf, "Stopping motor");
	Timer::Abort();
	Detector::Disable();
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
	state = State::Stopped;
	Timer::Schedule(500000, Idle);
}
