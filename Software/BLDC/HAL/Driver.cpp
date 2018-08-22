#include "Driver.hpp"

#include "lowlevel.hpp"
#include "Detector.hpp"
#include "Timer.hpp"

using namespace HAL::BLDC;

static Driver::IncCallback IncCB;
static void* IncPtr;

static uint8_t CommutationStep;

enum class State : uint8_t {
	Stopped,
	Starting,
	Running,
};

static State state;
static uint8_t StartStep;
static constexpr uint32_t StartSequence[] = {150000, 100000, 50000, 20000};
static constexpr uint8_t StartSequenceLength = sizeof(StartSequence)/sizeof(StartSequence[0]);

static void SetStep(uint8_t step) {
	switch (step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		Detector::Enable(Detector::Phase::B);
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		Detector::Enable(Detector::Phase::A);
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		Detector::Enable(Detector::Phase::C);
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		Detector::Enable(Detector::Phase::B);
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		Detector::Enable(Detector::Phase::A);
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		Detector::Enable(Detector::Phase::C);
		break;
	}
}

static void NextStartStep() {
	CommutationStep = (CommutationStep + 1) % 6;
	SetStep(CommutationStep);
	// Schedule next start step
	StartStep++;
	if (StartStep >= StartSequenceLength) {
		// TODO Start attempt failed
		return;
	}

	Timer::Schedule(StartSequence[StartStep - 1], NextStartStep);
}

static void CrossingCallback(uint32_t usSinceLast, uint32_t timeSinceCrossing) {
	if (state == State::Starting) {
		state = State::Running;
		// abort next scheduled start step
		Timer::Abort();
	} else if (IncCB) {
		// motor is already running, report back crossing intervals to controller
		IncCB(IncPtr, usSinceLast);
	}

	// Disable detector until next commutation step
	Detector::Disable();
	CommutationStep = (CommutationStep + 1) % 6;
	// Calculate time until next 30° rotation
	uint32_t TimeToNextCommutation = usSinceLast / 2 - timeSinceCrossing;
	Timer::Schedule(TimeToNextCommutation, []() {
		SetStep(CommutationStep);
	});
}

HAL::BLDC::Driver::Driver() {
	LowLevel::SetPWM(0);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);

	Detector::Init(CrossingCallback);
	Detector::Disable();

	CommutationStep = 0;

	state = State::Stopped;
}

void HAL::BLDC::Driver::SetPWM(uint16_t promille) {
	LowLevel::SetPWM(
			LowLevel::MaxPWM / 2
					+ (uint32_t) LowLevel::MaxPWM / 2 * promille / 1000);
}

void HAL::BLDC::Driver::InitiateStart() {
	state = State::Starting;
	StartStep = 0;
	NextStartStep();
}

void HAL::BLDC::Driver::RegisterIncCallback(IncCallback c, void* ptr) {
	IncCB = c;
	IncPtr = ptr;
}

void HAL::BLDC::Driver::RegisterADCCallback(ADCCallback c, void* ptr) {
}
