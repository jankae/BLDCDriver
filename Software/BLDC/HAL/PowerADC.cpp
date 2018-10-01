#include "PowerADC.hpp"

#include "stm32f3xx_hal.h"

#include "Logging.hpp"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim15;
extern OPAMP_HandleTypeDef hopamp2;

static constexpr uint16_t BufferSize = 500;

uint16_t buf[BufferSize];
static bool voltageInLimits;
static uint16_t VoltageLimit;


void HAL::BLDC::PowerADC::PrintBuffer() {
	for(uint16_t i = 0;i<BufferSize;i++) {
		Log::Uart(Log::Lvl::Inf, "%d", buf[i]);
		HAL_Delay(1);
	}
}

static void AdjustWatchdog() {
	constexpr uint16_t voltageDividerRHigh = 4700;
	constexpr uint16_t voltageDividerRLow = 1000;
	constexpr uint16_t ADC_Ref = 3300;

	constexpr uint16_t Hysteresis_mV = 500;

	uint16_t mV = VoltageLimit;

	if(voltageInLimits) {
		mV += Hysteresis_mV;
	} else {
		mV -= Hysteresis_mV;
	}

	const uint32_t mV_ADC = mV * voltageDividerRLow
			/ (voltageDividerRLow + voltageDividerRHigh);
	const uint32_t threshold = mV_ADC * 4096 / ADC_Ref;
	HAL_ADC_Stop_DMA(&hadc2);
	ADC_AnalogWDGConfTypeDef wdt;
	wdt.Channel = ADC_CHANNEL_1;
	if (voltageInLimits) {
		// watch for exceeding limits
		wdt.LowThreshold = 0;
		wdt.HighThreshold = threshold;
	} else {
		// voltage already outside limits -> watch for return to limits
		wdt.LowThreshold = threshold;
		wdt.HighThreshold = 4095;
	}
	wdt.ITMode = ENABLE;
	wdt.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	wdt.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	HAL_ADC_AnalogWDGConfig(&hadc2, &wdt);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) buf, BufferSize);
}

void HAL::BLDC::PowerADC::Pause() {
	HAL_ADC_Stop_DMA(&hadc2);
}

void HAL::BLDC::PowerADC::Resume() {
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) buf, BufferSize);
	AdjustWatchdog();
}


void HAL::BLDC::PowerADC::SetMaximumVoltageThreshold(uint16_t mV) {
	VoltageLimit = mV;
	AdjustWatchdog();
}

bool HAL::BLDC::PowerADC::VoltageWithinLimits() {
	/* check the analog watchdog out bit */
	return voltageInLimits;
}

void HAL::BLDC::PowerADC::Init() {
	voltageInLimits = true;
	SetMaximumVoltageThreshold(16000);
	HAL_OPAMP_Start(&hopamp2);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) buf, BufferSize);
	HAL_TIM_Base_Start(&htim15);
}

void HAL::BLDC::PowerADC::DMAComplete() {
}

void HAL::BLDC::PowerADC::DMAHalfComplete() {
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC2) {
		/* voltage window watchdog triggered */
		voltageInLimits = !voltageInLimits;
		AdjustWatchdog();
		if(voltageInLimits) {
			Log::Uart(Log::Lvl::Inf, "Supply voltage back within limit");
		} else {
			Log::Uart(Log::Lvl::Err, "Supply voltage too high");
		}
	}
}



