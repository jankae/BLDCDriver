#include "InductanceSensing.hpp"

#include "lowlevel.hpp"
#include "stm32f3xx_hal.h"
#include "PowerADC.hpp"
#include "Detector.hpp"
#include "Logging.hpp"

using namespace HAL::BLDC;

extern ADC_HandleTypeDef hadc2;

static constexpr uint16_t pulseLength = 50;
static constexpr uint8_t BaseClkMHz = 32;

static constexpr uint8_t CurrentADCClockMHz = 64;
static constexpr uint16_t CurrentADCSamplingCycles = 8;

static uint8_t stepCnt;
static volatile bool done;

uint16_t I[6];

static void SetPhases(uint8_t step) {
	switch(step) {
	case 0:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstHigh);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstLow);
		break;
	case 1:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		break;
	case 2:
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstHigh);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstLow);
		break;
	case 3:
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		break;
	case 4:
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstHigh);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstLow);
		break;
	case 5:
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		break;
	case 6:
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstHigh);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::ConstLow);
		break;
	case 7:
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
		break;
	case 8:
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstHigh);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstLow);
		break;
	case 9:
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		break;
	case 10:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstHigh);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::ConstLow);
		break;
	case 11:
		LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
		LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);
		break;
	}
}

/*
 * Configures PWM and ADC sampling for inductance sensing
 */
static void ConfigureHardware(uint32_t *SettingBuffer, uint16_t *CurrentBuf) {
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::B, LowLevel::State::Idle);
	LowLevel::SetPhase(LowLevel::Phase::C, LowLevel::State::Idle);

	PowerADC::Pause();
	Detector::Disable();

    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0 ,0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	// halt and reset all relevant timers
	// TIM2: base clk source
	// TIM1: PWM generation and phase voltage ADC timing
	// TIM15: Current ADC timing
	TIM2->CR1 &= ~TIM_CR1_CEN;

	TIM1->CNT = 0;
	TIM2->CNT = 0;
	TIM15->CNT = 0;

	// configure PWM to a cycle length of pulseLength to adjust phase voltage ADC sampling rate
	constexpr uint16_t PWMPeriod = pulseLength * BaseClkMHz;
	SettingBuffer[0] = TIM1->ARR;
	TIM1->ARR = PWMPeriod - 1;

	// Setup current ADC sampling rate
	SettingBuffer[1] = TIM15->ARR;
	// sample synchronous with every second step
	TIM15->ARR = (PWMPeriod * 2) - 1;//BaseClkMHz / currentSampleRateMHz - 1;

	// Update all timers
	TIM1->EGR = TIM_EGR_UG;
	TIM2->EGR = TIM_EGR_UG;
	TIM15->EGR = TIM_EGR_UG;

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) CurrentBuf, 6);

	// start sampling slightly before the next step starts
	TIM15->CNT = PWMPeriod + CurrentADCSamplingCycles * BaseClkMHz / CurrentADCClockMHz;

	// clear timer1 update flag
	TIM1->SR &= ~TIM_SR_UIF;
	// enable interrupt and start timer
	TIM1->DIER = TIM_DIER_UIE;

	// charge boot capacitor as phase A will be high in the first induction sensing step
	LowLevel::SetPhase(LowLevel::Phase::A, LowLevel::State::ConstLow);
	HAL_Delay(1);

	// Start the whole process by enabling the master clock
	TIM2->CR1 |= TIM_CR1_CEN;
	stepCnt = 0;
	SetPhases(stepCnt);
}

/*
 * Reverts configuration state previous to inductance sensing
 */
static void ReconfigureHardware(uint32_t *SettingBuffer) {
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
	HAL_ADC_Stop_DMA(&hadc2);

	// halt and reset all relevant timers
	// TIM2: base clk source
	// TIM1: Step interrupt generation
	// TIM15: Current ADC timing
	TIM2->CR1 &= ~TIM_CR1_CEN;

	TIM1->CNT = 0;
	TIM2->CNT = 0;
	TIM15->CNT = 0;

	// Restore changed timer settings
	TIM1->ARR = SettingBuffer[0];
	TIM15->ARR = SettingBuffer[1];

	// Update all timers
	TIM1->EGR = TIM_EGR_UG;
	TIM2->EGR = TIM_EGR_UG;
	TIM15->EGR = TIM_EGR_UG;

	// enable the base clock again
	TIM2->CR1 |= TIM_CR1_CEN;

	PowerADC::Resume();
}

uint16_t HAL::BLDC::InductanceSensing::RotorPosition() {
	done = false;
	uint32_t buf[2];
//	uint16_t I[6];
	ConfigureHardware(buf, I);
	while (!done) {

	}
	ReconfigureHardware(buf);

	// compares inverted with respect to paper as ADC measures lower values for higher currents
	bool I_II = I[0] < I[1];
	bool III_IV = I[3] < I[2];
	bool V_VI = I[4] < I[5];

	uint8_t section = 0;

	if(I_II && !III_IV && !V_VI) {
		section = 1;
	} else if(I_II && III_IV && !V_VI) {
		section = 2;
	} else if(I_II && III_IV && V_VI) {
		section = 3;
	} else if(!I_II && III_IV && V_VI) {
		section = 4;
	} else if(!I_II && !III_IV && V_VI) {
		section = 5;
	} else if(!I_II && !III_IV && !V_VI) {
		section = 6;
	}
	return section;
}

extern "C" {
void TIM1_UP_TIM16_IRQHandler() {
	// clear interrupt flag
	TIM1->SR &= ~TIM_SR_UIF;
	stepCnt++;
	if (stepCnt >= 12) {
		// all steps done
		// disable this interrupt
		TIM1->DIER &= ~TIM_DIER_UIE;
		done = true;
	} else {
		SetPhases(stepCnt);
	}
}
}
