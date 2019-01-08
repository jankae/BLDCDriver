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

static constexpr int ADCBufferLength = 3;

static uint16_t ADCBuf[ADCBufferLength];

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
//			Log::Uart(Log::Lvl::Wrn, "Switch to idling due to high DC bus voltage");
		}
	}

	PWMperiodCnt++;
	cnt++;

	switch(state) {
	case State::Idle_Braking:
		if(PowerADC::VoltageWithinLimits()) {
			// the DC bus voltage dropped sufficiently to resume powered operation
//			Log::Uart(Log::Lvl::Inf, "Resume powered operation");
			NextState(State::Powered_PreZero);
			break;
		}
		/* no break */
	case State::Idle:
	{
		// do not apply any voltages to the phase terminals
		SetIdle();
		CommutationCycles.fill(2 * Defines::PWM_Frequency);
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
//				Log::Uart(Log::Lvl::Dbg, "Below threshold: idle tracking inactive");
				valid = false;
			} else if (!valid
					&& max > idleDetectionThreshold + idleDetectionHysterese) {
//				Log::Uart(Log::Lvl::Dbg, "Above threshold: idle tracking active");
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
				if (state == State::Idle_Braking) {
					// stopped completely while idle braking, prevent the driver
					// from switching back to powered state
					NextState(State::Idle);
				}
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
	case State::AlignAndGo:
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
			Log::Uart(Log::Lvl::Inf, "...motor aligned");
			if (state == State::AlignAndGo) {
				IncRotorPos();
				LowLevel::SetPWM(minPWM);
				SetStep(RotorPos);
				NextState(State::Starting);
			} else {
				SetIdle();
				NextState(State::Idle);
			}
		}
	}
		break;
	case State::Starting:
		SetStep(RotorPos);
		NextState(State::Powered_PreZero);
		break;
	case State::Powered_PreZero:
	{
		constexpr uint16_t nPulsesSkip = 1;
		constexpr uint16_t ZeroThreshold = 10;
		constexpr uint32_t timeoutThresh = CommutationTimeoutms
				* Defines::PWM_Frequency / 1000;

		const uint16_t supply = data[(int) nPhaseHigh];

//		uint16_t skip = nPulsesSkip;
//		if (state == State::Starting) {
//			// this is the first commutation from a stopped position
//			// wait a little bit longer until enabling detector
//			skip += 2;
//		}

		static uint32_t integral;

		if (cnt == 1) {
			SetStep(RotorPos);
			integral = 0;
		}
		if (true) {
			if(cnt >= timeoutThresh) {
				// failed to detect the next commutation in time, motor probably stalled
				Log::Uart(Log::Lvl::Wrn, "Commutation timed out");
				IncRotorPos();
				IncRotorPos();
				DetectorArmed = false;
				NextState(State::Powered_PreZero);
			}

			const uint16_t phase = data[(int) nPhaseIdle];

			const uint16_t min = supply / 10;
			if(phase < min || phase + min > supply) {
				break;
			}

			bool rising = (dir == Direction::Forward) ^ (RotorPos & 0x01);
			const uint16_t zero = supply * mot->ZeroCal[RotorPos] / 65536;;
			int16_t compare = phase - zero;

			constexpr uint32_t integralLimit = 750;
			uint32_t limit = integralLimit;
			if (state == State::Starting) {
				limit *= 50;
			}
			if(!rising) {
				compare = -compare;
			}
			if(compare > 0) {
				integral += compare;
				if(integral >= limit) {
					CommutationCycles[RotorPos] = cnt;
					IncRotorPos();
					SetStep(RotorPos);
					if(cnt <= 2) {
						// check other commutation cycles cnt to detect backfiring motor
						uint16_t sum = 0;
						for(auto i : CommutationCycles) {
							sum += i;
						}
						Log::Uart(Log::Lvl::Wrn, "Sum %d", sum);
						if (sum > 6 * cnt * 100) {
							Log::Uart(Log::Lvl::Wrn,
									"Detected backfiring motor, aligning");
							NextState(State::AlignAndGo);
							break;
						}
					}
					NextState(State::Powered_PreZero);
					HAL_GPIO_TogglePin(TRIGGER_GPIO_Port, TRIGGER_Pin);
				}
			}

//			if (DetectorArmed) {
//				if ((compare <= 0 && !rising)
//						|| (compare >= ZeroThreshold && rising)) {
//					// crossing detected
//					timeToZero = cnt;
//					CommutationCycles[RotorPos] = cnt;
//					NextState(State::Powered_PastZero);
//				}
//			} else {
//				if ((compare <= 0 && rising)
//						|| (compare > 0 && !rising)) {
//					DetectorArmed = true;
//				}
//			}
		}
	}
		break;
	case State::Powered_PastZero:
	{
		if(cnt >= timeToZero) {
			// next commutation is due
			IncRotorPos();
			SetStep(RotorPos);
			NextState(State::Powered_PreZero);
			HAL_GPIO_TogglePin(TRIGGER_GPIO_Port, TRIGGER_Pin);
		}
	}
		break;
	case State::Calibrating: {
		if (cnt == 1) {
			HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
		}
		constexpr uint16_t periodsPerMeasurement = 5;
		const uint8_t measurement = cnt / periodsPerMeasurement;
		const uint8_t intCycle = cnt % periodsPerMeasurement;
		if (intCycle == 1) {
			SetIdle();
			if (measurement > 0) {
				mot->ZeroCal[measurement - 1] = data[(int) nPhaseIdle] * 65536UL
						/ data[(int) nPhaseHigh];
			}
			if (measurement >= 6) {
				HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin,
						GPIO_PIN_RESET);
				// calibration completed
				NextState(State::Idle);
			}
		} else if (intCycle == 0) {
			LowLevel::SetPWM(minPWM);
			SetStep(measurement - 1);
		}
	}
		break;
	case State::Testing: {
		constexpr uint16_t minHighVoltage = 2000;
		/* Set output driver according to test step and check measured phases from previous step */
		switch(cnt) {
		case 1:
			result = (int) TestResult::OK;
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstHigh);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			break;
		case 2:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstHigh);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			if(data[(int) LowLevel::Phase::A] < minHighVoltage) {
				result = (int) TestResult::Failure;
			}
			break;
		case 3:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstHigh);
			if(data[(int) LowLevel::Phase::B] < minHighVoltage) {
				result = (int) TestResult::Failure;
			}
			break;
		case 4:
			LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
			LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
			if(data[(int) LowLevel::Phase::C] < minHighVoltage) {
				result = (int) TestResult::Failure;
			}
			if (result == (int) TestResult::OK) {
				/* output driver seems to work, check for motor */
				if (data[(int) LowLevel::Phase::A] < minHighVoltage
						|| data[(int) LowLevel::Phase::B] < minHighVoltage) {
					result = (int) TestResult::NoMotor;
				}
			}
			NextState(State::Idle);
			break;
		}
	}
		break;
	case State::MeasuringResistance: {
		// apply low pwm values to all the phases, measure current after RL step response
		constexpr uint32_t EstimatedTimeConstant_us = 1000;
		// after 6 time constants the current has reached 99.7% of its final value, that is close enough
		constexpr uint32_t WaitTime_us = 6 * EstimatedTimeConstant_us;
		constexpr uint16_t WaitCycles = WaitTime_us * Defines::PWM_Frequency
				/ 1000000UL;
		constexpr uint16_t ResistancePWM = 150;
		constexpr uint8_t StepSequence[6] = { 0, 3, 1, 4, 2, 5 };
		uint8_t step = cnt / WaitCycles;
		uint16_t cycle = cnt % WaitCycles;
		if (cycle == 1) {
			if (step <= 6) {
				// energize the next phase
				LowLevel::SetPWM(ResistancePWM);
				SetStep(StepSequence[step]);
			}
			if (step > 0) {
				// measure current and voltage from last cycle
				auto m = PowerADC::GetInstant();
				// calculate voltage and current at motor based on pwm value
				uint32_t Vmotor = m.voltage * ResistancePWM / 1000;
				int32_t Imotor = m.current * 1000 / ResistancePWM;
				uint32_t resistance_mR = (uint64_t) Vmotor * 1000 / Imotor;
//				Log::Uart(Log::Lvl::Dbg, "Resistance step %d: %lumR", step - 1, resistance_mR);
				result += resistance_mR;
			}
			if (step >= 6) {
				// measured all phases
				result /= 6;
				NextState(State::Idle);
			}
		}
	}
		break;
	default:
		break;
	}
}

HAL::BLDC::Driver::Driver(Data *d) {
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
	mot = d;
	// check if data is plausible
	bool plausible = true;
	for(auto i : mot->ZeroCal) {
		constexpr uint16_t maxDiff = 10000;
		if(i<32768-maxDiff || i > 32768+maxDiff) {
			plausible = false;
			break;
		}
	}
	if (!plausible) {
		mot->ZeroCal.fill(32768);
		Log::Uart(Log::Lvl::Wrn, "Implausible motor calibration data, resetting");
	}
#ifdef DRIVER_BUFFER
	buffer.clear();
#endif

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
	/*
	 * The DMA interrupt -> DMA handler -> ADC handler -> Dispatcher -> Driver state machine
	 * flow that CubeMX generates is too slow for the driver to update the PWM values in the
	 * same cycle. As a workaround, the state machine is directly called from the corresponding
	 * ISR and the half complete interrupt is not needed anymore
	 *
	 * TODO get rid of the HAL overhead and implement the ADC + DMA directly inside the driver
	 */
	DMA1_Channel1->CCR &= ~DMA_CCR_HTIE;

	Inst = this;

	Log::Uart(Log::Lvl::Inf, "Created driver object");
}

void HAL::BLDC::Driver::SetPWM(int16_t promille) {
	LowLevel::SetPWM(promille);
}

Driver::State HAL::BLDC::Driver::GetState() {
	return state;
}

void HAL::BLDC::Driver::FreeRunning() {
	stateBuf = State::Idle;
}

void HAL::BLDC::Driver::Stop() {
	stateBuf = State::Stopped;
}

extern "C" {
void DriverInterrupt() {
	if (Driver::Inst) {
		Driver::Inst->NewPhaseVoltages(&ADCBuf[0]);
	}
}
}

void HAL::BLDC::Driver::InitiateStart() {
	if (IsRunning()) {
		// nothing to do, motor already running
		return;
	}
	stateBuf = State::Idle;
	if (RotorPos == -1) {
		Log::Uart(Log::Lvl::Dbg, "Starting motor from unknown position: initial position detection");
		// rotor position not known at the moment, detect using inductance sensing
		HAL_ADC_Stop_DMA(&hadc1);
		uint8_t pos = InductanceSensing::RotorPosition();
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
		Log::Uart(Log::Lvl::Dbg, "Position: %d", pos);
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
			WhileStateEquals(State::Align);
			IncRotorPos();
		}
	}
	LowLevel::SetPWM(minPWM);
	DetectorArmed = false;
	SetStep(RotorPos);
	stateBuf = State::Starting;
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
	WhileStateEquals(State::Testing);
	return (TestResult) result;
}

bool HAL::BLDC::Driver::GotValidPosition() {
	return (RotorPos != -1);
}

uint32_t HAL::BLDC::Driver::WindingResistance() {
	stateBuf = State::MeasuringResistance;
	result = 0;
	WhileStateEquals(State::MeasuringResistance);
	return result;
}

void HAL::BLDC::Driver::IncRotorPos() {
	if (dir == Direction::Forward) {
		RotorPos = (RotorPos + 1) % 6;
	} else {
		RotorPos = (RotorPos + 5) % 6;
	}
	commutationCnt++;
}

bool HAL::BLDC::Driver::IsRunning() {
	return (*(volatile State*) &state == State::Powered_PastZero
			|| *(volatile State*) &state == State::Powered_PreZero);
}

uint16_t HAL::BLDC::Driver::GetRPMSmoothed() {
	uint32_t CommutationsPerSecond = (uint64_t) commutationCnt
			* Defines::PWM_Frequency / PWMperiodCnt;
	PWMperiodCnt = commutationCnt = 0;

	uint16_t rpm = CommutationsPerSecond * 60 / 6 / (MotorPoles / 2);

	return rpm;
}

uint16_t HAL::BLDC::Driver::GetRPMInstant() {
	if (IsRunning()) {
		uint32_t PWMPeriodSum = (CommutationCycles[0] + CommutationCycles[1]
				+ CommutationCycles[2] + CommutationCycles[3]
				+ CommutationCycles[4] + CommutationCycles[5]);
		uint32_t CommutationsPerSecond = 6 * Defines::PWM_Frequency
				/ PWMPeriodSum;

		uint16_t rpm = CommutationsPerSecond * 60 / 6 / (MotorPoles / 2);

		return rpm;
	} else {
		return 0;
	}
}

HAL::BLDC::Driver::MotorData HAL::BLDC::Driver::GetData() {
	MotorData d;
	auto m = PowerADC::GetSmoothed();
	d.rpm = GetRPMInstant();
	d.current = m.current;
	d.voltage = m.voltage;

	return d;
}

void HAL::BLDC::Driver::Calibrate() {
	if(state != State::Idle) {
		Log::Uart(Log::Lvl::Err, "Unable to calibrate non-idling motor");
		return;
	}
	uint32_t sum[6] = { 0, 0, 0, 0, 0, 0 };
	for (uint8_t i = 0; i < 6; i++) {
		RotorPos = i;
		stateBuf = State::Align;
		WhileStateEquals(State::Align);
		stateBuf = State::Calibrating;
		WhileStateEquals(State::Calibrating);
		for (uint8_t j = 0; j < 6; j++) {
			sum[j] += mot->ZeroCal[j];
		}
	}

	// measured offsets over one electrical rotation, average
	for (uint8_t j = 0; j < 6; j++) {
		mot->ZeroCal[j] = sum[j] / 6;
	}
	Log::Uart(Log::Lvl::Dbg, "Result: %d %d %d %d %d %d", mot->ZeroCal[0],
			mot->ZeroCal[1], mot->ZeroCal[2], mot->ZeroCal[3], mot->ZeroCal[4],
			mot->ZeroCal[5]);
}

void HAL::BLDC::Driver::WhileStateEquals(State s) {
	while (*(volatile State*) &stateBuf == s || *(volatile State*) &state == s)
		;
}
