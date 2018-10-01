#include "Driver.hpp"

#include "lowlevel.hpp"
#include "Logging.hpp"

#include "stm32f3xx_hal.h"
#include "stm32f303x8.h"
#include "InductanceSensing.hpp"
#include "fifo.hpp"
#include "PowerADC.hpp"
#include "Defines.hpp"

using namespace HAL::BLDC;

Driver* HAL::BLDC::Driver::Inst;

extern ADC_HandleTypeDef hadc1;

static constexpr int ADCBufferLength = 6;

static uint16_t ADCBuf[ADCBufferLength];

static Driver::IncCallback IncCB;
static void* IncPtr;

void HAL::BLDC::Driver::SetStep(uint8_t step) {
	switch (step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		nPhaseHigh = LowLevel::Phase::A;
		nPhaseIdle = LowLevel::Phase::B;
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Low);
		nPhaseHigh = LowLevel::Phase::B;
		nPhaseIdle = LowLevel::Phase::A;
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		nPhaseHigh = LowLevel::Phase::B;
		nPhaseIdle = LowLevel::Phase::C;
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		nPhaseHigh = LowLevel::Phase::C;
		nPhaseIdle = LowLevel::Phase::B;
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::High);
		nPhaseHigh = LowLevel::Phase::C;
		nPhaseIdle = LowLevel::Phase::A;
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::High);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Low);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		nPhaseHigh = LowLevel::Phase::A;
		nPhaseIdle = LowLevel::Phase::C;
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

	if (state == State::Powered_PastZero || state == State::Powered_PreZero) {
		if (!PowerADC::VoltageWithinLimits()) {
			// DC bus voltage too high, presumably due to regenerative braking */
			SetIdle();
			NextState(State::Idle_Braking);
			Log::Uart(Log::Lvl::Wrn, "Switch to idling due to high DC bus voltage");
		}
	}

	cnt++;

	switch(state) {
	case State::Idle_Braking:
		if(PowerADC::VoltageWithinLimits()) {
			// the DC bus voltage dropped sufficiently to resume powered operation
			Log::Uart(Log::Lvl::Inf, "Resume powered operation");
			NextState(State::Powered_PreZero);
			break;
		}
		/* no break */
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
		if (cnt > 10) {
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
		RotorPos = -1;
		break;
	case State::Align:
	{
		// align the rotor to a known position prior to starting the motor
		constexpr uint32_t msHold = 1000;
		constexpr uint32_t cntThresh = msHold * Defines::PWM_Frequency / 1000;
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
		constexpr uint16_t nPulsesSkip = 2;
		constexpr uint16_t ZeroThreshold = 10;
		constexpr uint32_t timeoutThresh = CommutationTimeoutms
				* Defines::PWM_Frequency / 1000;

		const uint16_t zero = data[(int) nPhaseHigh] / 2;
		static bool above = false;

		if (cnt == 1) {
			SetStep(RotorPos);
		} else if (cnt > nPulsesSkip) {
			if (zero > 100 && !above) {
				above = true;
				Log::Uart(Log::Lvl::Inf, "Switched to on phase sampling");
			} else if(zero < 100 && above) {
				above = false;
				Log::Uart(Log::Lvl::Inf, "Switched to off phase sampling");
			}

			bool rising = (dir == Direction::Forward) ^ (RotorPos & 0x01);
			int16_t compare = data[(int) nPhaseIdle] - zero;

			if (DetectorArmed) {
				if ((compare <= 0 && !rising)
						|| (compare >= ZeroThreshold && rising)) {
					// crossing detected
					timeToZero = cnt;
					NextState(State::Powered_PastZero);
				}
			} else {
				if ((compare <= 0 && rising)
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
	case State::Testing: {
		constexpr uint16_t minHighVoltage = 2000;
		/* Set output driver according to test step and check measured phases from previous step */
		switch(cnt) {
		case 1:
			testResult = TestResult::OK;
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstHigh);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			break;
		case 2:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstHigh);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			if(data[(int) LowLevel::Phase::A] < minHighVoltage) {
				testResult = TestResult::Failure;
			}
			break;
		case 3:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstHigh);
			if(data[(int) LowLevel::Phase::B] < minHighVoltage) {
				testResult = TestResult::Failure;
			}
			break;
		case 4:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			if(data[(int) LowLevel::Phase::C] < minHighVoltage) {
				testResult = TestResult::Failure;
			}
			if (testResult == TestResult::OK) {
				/* output driver seems to work, check for motor */
				if (data[(int) LowLevel::Phase::A] < minHighVoltage
						|| data[(int) LowLevel::Phase::B] < minHighVoltage) {
					testResult = TestResult::NoMotor;
				}
			}
			NextState(State::Idle);
			break;
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
	SetIdle();

	state = State::Idle;
	stateBuf = State::None;
	cnt = 0;
	dir = Direction::Forward;
	RotorPos = -1;
#ifdef DRIVER_BUFFER
	buffer.clear();
#endif

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);

	Inst = this;
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

void HAL::BLDC::Driver::FreeRunning() {
	stateBuf = State::Idle;
}

void HAL::BLDC::Driver::Stop() {
	stateBuf = State::Stopped;
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

void HAL::BLDC::Driver::SetIdle() {
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
}

HAL::BLDC::Driver::~Driver() {
	SetIdle();
	HAL_ADC_Stop_DMA(&hadc1);
	Inst = nullptr;
}

Driver::TestResult HAL::BLDC::Driver::Test() {
	stateBuf = State::Testing;
	while (*(volatile State*) &stateBuf == State::Testing
			|| *(volatile State*) &state == State::Testing)
		;
	return testResult;
}

bool HAL::BLDC::Driver::GotValidPosition() {
	return (RotorPos != -1);
}

void HAL::BLDC::Driver::IncRotorPos() {
	if (dir == Direction::Forward) {
		RotorPos = (RotorPos + 1) % 6;
	} else {
		RotorPos = (RotorPos + 5) % 6;
	}
}

bool HAL::BLDC::Driver::IsRunning() {
	return (*(volatile State*) &state == State::Powered_PastZero
			|| *(volatile State*) &state == State::Powered_PreZero);
}
