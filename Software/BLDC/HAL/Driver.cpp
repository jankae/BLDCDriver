#include "Driver.hpp"

#include "lowlevel.hpp"
#include "Detector.hpp"
#include "Timer.hpp"
#include "Logging.hpp"

#include "stm32f3xx_hal.h"
#include "stm32f303x8.h"
#include "InductanceSensing.hpp"
#include "fifo.hpp"

using namespace HAL::BLDC;

Driver* HAL::BLDC::Driver::Inst;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

static constexpr int ADCBufferLength = 6;

static uint16_t ADCBuf[ADCBufferLength];
static uint16_t *ValidBuf = ADCBuf;

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

void HAL::BLDC::Driver::SetStep(uint8_t step) {
//	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
//	HAL_GPIO_TogglePin(TRIGGER_GPIO_Port, TRIGGER_Pin);
	switch (step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		nPhaseHigh = LowLevel::Phase::A;
		nPhaseIdle = LowLevel::Phase::B;
//		Detector::SetPhase(Detector::Phase::B, true);
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		nPhaseHigh = LowLevel::Phase::B;
		nPhaseIdle = LowLevel::Phase::A;
//		Detector::SetPhase(Detector::Phase::A, false);
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		nPhaseHigh = LowLevel::Phase::B;
		nPhaseIdle = LowLevel::Phase::C;
//		Detector::SetPhase(Detector::Phase::C, true);
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		nPhaseHigh = LowLevel::Phase::C;
		nPhaseIdle = LowLevel::Phase::B;
//		Detector::SetPhase(Detector::Phase::B, false);
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		nPhaseHigh = LowLevel::Phase::C;
		nPhaseIdle = LowLevel::Phase::A;
//		Detector::SetPhase(Detector::Phase::A, true);
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		nPhaseHigh = LowLevel::Phase::A;
		nPhaseIdle = LowLevel::Phase::C;
//		Detector::SetPhase(Detector::Phase::C, false);
		break;
	}
}

#define NextState(s) do { state = s; cnt = 0;} while(0);

//#define DRIVER_BUFFER
#ifdef DRIVER_BUFFER
Fifo<uint16_t, 1500> buffer __attribute__ ((section (".ccmram")));
#endif

void HAL::BLDC::Driver::NewPhaseVoltages(uint16_t *data) {
	if (stateBuf != State::None) {
		if (state != stateBuf) {
			// state was changed by driver funtion call
			NextState(stateBuf);
			stateBuf = State::None;
		}
	}

	cnt++;

	switch(state) {
	case State::Idle:
	{
		// do not apply any voltages to the phase terminals
		SetIdle();
#ifdef DRIVER_BUFFER
		if(cnt % 30 == 0) {
			if(buffer.getLevel() > 0) {
				uint16_t A = 0, B = 0, C = 0;
				buffer.dequeue(A);
//				buffer.dequeue(B);
//				buffer.dequeue(C);
				Log::Uart(Log::Lvl::Inf, " %d ", A);
			}
		}
#endif
		if (cnt > 50) {
			static constexpr uint16_t idleDetectionThreshold = 25;
			static constexpr uint16_t idleDetectionHysterese = 15;
			// attempt to track the rotor position from the induced voltages
			uint16_t max = data[0];
			if (data[1] > max) {
				max = data[1];
			}
			if (data[2] > max) {
				max = data[2];
			}
			static bool valid = false;
			if (valid
					&& max < idleDetectionThreshold - idleDetectionHysterese) {
				// induced voltage too small to reliable track position, assume motor has stopped
				Log::Uart(Log::Lvl::Inf, "Below threshold: idle tracking inactive");
				valid = false;
			} else if (!valid
					&& max > idleDetectionThreshold + idleDetectionHysterese) {
				Log::Uart(Log::Lvl::Inf, "Above threshold: idle tracking active");
				valid = true;
			}
			if (valid) {
				const auto A = data[(int) LowLevel::Phase::A];
				const auto B = data[(int) LowLevel::Phase::B];
				const auto C = data[(int) LowLevel::Phase::C];
				if (A > B && B >= C) {
					RotorPos = 0;
				} else if (B > A && A >= C) {
					RotorPos = 1;
				} else if (B > C && C >= A) {
					RotorPos = 2;
				} else if (C > B && B >= A) {
					RotorPos = 3;
				} else if (C > A && A >= B) {
					RotorPos = 4;
				} else if (A > C && C >= B) {
					RotorPos = 5;
				}
			} else {
				RotorPos = -1;
			}
		}
	}
		break;
	case State::Stopped:
		// keep the motor stopped by pulling all phases to ground
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstLow);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstLow);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstLow);
		break;
	case State::Align:
	{
		// align the rotor to a known position prior to starting the motor
		constexpr uint32_t msHold = 1000;
		constexpr uint32_t cntThresh = msHold * HzPWM / 1000;
		constexpr uint16_t alignPWM = minPWM / 2;
		if (cnt == 1) {
			Log::Uart(Log::Lvl::Inf, "Aligning motor...");
			LowLevel::SetPWM(alignPWM);
			SetStep(RotorPos);
		} else if(cnt > cntThresh) {
			Log::Uart(Log::Lvl::Inf, "Powering motor...");
			IncRotorPos();
			LowLevel::SetPWM(minPWM);
			NextState(State::Powered_PreZero);
		}
	}
		break;
	case State::Powered_PreZero:
	{
		constexpr uint16_t nPulsesSkip = 4;
		constexpr uint16_t ZeroThreshold = 10;
		constexpr uint32_t timeoutThresh = CommutationTimeoutms * HzPWM / 1000;

		if (cnt == 1) {
			SetStep(RotorPos);
		} else if (cnt > nPulsesSkip) {
			bool rising = (dir == Direction::Forward) ^ (RotorPos & 0x01);
			uint16_t compare = data[(int) nPhaseIdle];

			if (DetectorArmed) {
				if ((compare == 0 && !rising)
						|| (compare >= ZeroThreshold && rising)) {
					// crossing detected
					timeToZero = cnt;
					NextState(State::Powered_PastZero);
				}
			} else {
				if ((compare == 0 && rising)
						|| (compare > 0 && !rising)) {
					DetectorArmed = true;
				}
			}
			if(cnt >= timeoutThresh) {
				// failed to detect the next commutation in time, motor probably stalled
				Log::Uart(Log::Lvl::Wrn, "Commutation timed out");
				RotorPos = -1;
				NextState(State::Idle);
			}
		}
	}
	break;
	case State::Powered_PastZero:
	{
		if(cnt >= timeToZero) {
			// next commutation is due
			IncRotorPos();
			NextState(State::Powered_PreZero);
		}
	}
		break;
	case State::Calibrating:
	{
		// manually rotate through all commutation steps and record zero correction factors
		constexpr uint16_t cycleLength = 30000;
		constexpr uint16_t sampleLength = 5000;
		uint8_t step = cnt / cycleLength;
		uint16_t phase = cnt % cycleLength;
		static uint32_t sumHigh;
		static uint32_t sumMiddle;
		if (phase == 1) {
			if (step > 0) {
				// not the first calibration step, calculate last sampled zero factor
				ZeroCal[step - 1] = (uint64_t) sumMiddle * 65536UL / sumHigh;
			}
			if (step < 6) {
				Log::Uart(Log::Lvl::Dbg, "Calibration step %d/6", step+1);
				sumHigh = 0;
				sumMiddle = 0;
				LowLevel::SetPWM(minPWM);
				SetStep(step);
			} else {
				// calibration completed
				Log::Uart(Log::Lvl::Inf, "Calibration completed");
				Log::Uart(Log::Lvl::Dbg, "Result: %d %d %d %d %d %d",
						ZeroCal[0], ZeroCal[1], ZeroCal[2], ZeroCal[3],
						ZeroCal[4], ZeroCal[5]);
				SetIdle();
				NextState(State::Idle);
			}
		} else if(phase > cycleLength - sampleLength) {
			sumHigh += data[(int) nPhaseHigh];
			sumMiddle += data[(int) nPhaseIdle];
		}
	}
		break;
	default:
		break;
	}
}

HAL::BLDC::Driver::Driver() {
	if(Inst) {
		Log::Uart(Log::Lvl::Crt, "Attempted to create second driver object");
		return;
	}
	LowLevel::SetPWM(0);
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);

//	Detector::Disable();

	CommutationStep = 0;

	state = State::Idle;
	stateBuf = State::None;
	cnt = 0;
	ZeroCal.fill(32768);
	ZeroCal = {36757, 36286, 34064, 29987 ,29523, 32087};
	dir = Direction::Reverse;
	RotorPos = -1;
#ifdef DRIVER_BUFFER
	buffer.clear();
#endif

	// start sampling
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);

	constexpr uint32_t ADCClockMHz = 64;
	constexpr uint32_t TimerClockMHz = 32;
	constexpr uint16_t ADCSamplingCycles = 5;
	constexpr uint16_t ADCOverallCycles = ADCBufferLength/2 * (ADCSamplingCycles + 12);
	constexpr uint16_t OverallTimerCycles = ADCOverallCycles * TimerClockMHz / ADCClockMHz;
	constexpr uint16_t minPWMValTimCnt = 1600 * minPWM / 1000;

//	TIM1->CCR4 = minPWMValTimCnt - OverallTimerCycles;
	TIM1->CCR4 = 1600 - OverallTimerCycles;
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

	Inst = this;

//	Detector::EnableIdleTracking(IdleTrackingCB);
}

void HAL::BLDC::Driver::SetPWM(int16_t promille) {
	LowLevel::SetPWM(promille);
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
	stateBuf = State::Idle;
}

void HAL::BLDC::Driver::Stop() {
	Log::Uart(Log::Lvl::Inf, "Stopping motor");
	Timer::Abort();
//	Detector::Disable();
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
	state = State::Stopped;
	Timer::Schedule(500000, Idle);
}

void HAL::BLDC::Driver::DMAComplete() {
	if (Inst) {
		Inst->NewPhaseVoltages(&ADCBuf[ADCBufferLength / 2]);
	}
}

void HAL::BLDC::Driver::DMAHalfComplete() {
	if (Inst) {
		Inst->NewPhaseVoltages(&ADCBuf[0]);
	}
}

void HAL::BLDC::Driver::InitiateStart() {
	stateBuf = State::Idle;
	if (RotorPos == -1) {
		Log::Uart(Log::Lvl::Inf, "Starting motor from unknown position: initial position detection");
		// rotor position not known at the moment, detect using inductance sensing
		HAL_ADC_Stop_DMA(&hadc1);
		uint8_t pos = InductanceSensing::RotorPosition();
//		pos = 0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
		Log::Uart(Log::Lvl::Inf, "Position: %d", pos);
		if (pos > 0) {
			if(dir == Direction::Forward) {
				RotorPos = (9 - pos) % 6;
			} else {
				RotorPos = (11 - pos) % 6;
			}
		} else {
			// unable to determine rotor position, use align and go
			Log::Uart(Log::Lvl::Wrn, "Unable to determine position, fall back to align and go");
			RotorPos = 0;
			stateBuf = State::Align;
			return;
		}
	}
	LowLevel::SetPWM(minPWM);
	DetectorArmed = false;
	stateBuf = State::Powered_PreZero;
}

void HAL::BLDC::Driver::ZeroCalibration() {
	Log::Uart(Log::Lvl::Inf, "Starting Zero Detection calibration");
	stateBuf = State::Calibrating;
}

bool HAL::BLDC::Driver::IsCalibrating() {
	return (state == State::Calibrating) || (stateBuf == State::Calibrating);
}

void HAL::BLDC::Driver::SetIdle() {
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
	LowLevel::SetPWM(0);
}

HAL::BLDC::Driver::~Driver() {
	SetIdle();
	HAL_ADC_Stop_DMA(&hadc1);
	Inst = nullptr;
}

void HAL::BLDC::Driver::IncRotorPos() {
	if (dir == Direction::Forward) {
		RotorPos = (RotorPos + 1) % 6;
	} else {
		RotorPos = (RotorPos + 5) % 6;
	}
}
