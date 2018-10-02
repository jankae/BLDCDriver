#include "PowerADC.hpp"

#include "stm32f3xx_hal.h"

#include "critical.hpp"
#include "Defines.hpp"
#include "Logging.hpp"

using namespace HAL::BLDC;

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim15;
extern OPAMP_HandleTypeDef hopamp2;

static constexpr uint16_t samplesPerPWMPeriod = Defines::ADC_SampleratePower
		/ Defines::PWM_Frequency;
static constexpr uint16_t BufferSize = samplesPerPWMPeriod
		* 2		// two measurements per sample (voltage + current)
		* 2;	// double buffer for the DMA

static uint16_t buf[BufferSize];
static uint16_t *validBuf = buf;
static bool voltageInLimits;
static uint16_t VoltageLimit;

static uint16_t currentZero = 2048;

static uint32_t voltage = 0;
static int32_t current = 0;
static uint64_t voltage_sum = 0;
static int64_t current_sum = 0;
static uint32_t samples = 0;

void HAL::BLDC::PowerADC::PrintBuffer() {
	for(uint16_t i = 0;i<BufferSize;i++) {
		Log::Uart(Log::Lvl::Inf, "%d", buf[i]);
		HAL_Delay(1);
	}
}

static void AdjustWatchdog() {
	constexpr uint16_t Hysteresis_mV = 500;

	uint16_t mV = VoltageLimit;

	if(voltageInLimits) {
		mV += Hysteresis_mV;
	} else {
		mV -= Hysteresis_mV;
	}

	const uint32_t mV_ADC = mV * Defines::voltageDividerRLow
			/ (Defines::voltageDividerRLow + Defines::voltageDividerRHigh);
	const uint32_t threshold = mV_ADC * 4096 / Defines::ADC_Ref;
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

PowerADC::Measurement HAL::BLDC::PowerADC::Get() {
	Measurement m;
	if(!samples) {
		m.voltage = voltage;
		m.current = current;
	} else {
		CriticalSection crit;
		m.voltage = voltage_sum / samples;
		m.current = current_sum / samples;
		voltage_sum = current_sum = samples = 0;
	}
	return m;
}

static void CalcVolCur() {
	/* calculate averages */
	uint32_t avgVol = 0;
	int32_t avgCur = 0;
	uint16_t maxCur = 0, minCur = 4096;
	for (uint16_t i = 0; i < BufferSize / 4; i++) {
		const uint16_t vol = validBuf[i * 2 + 1];
		const uint16_t cur = validBuf[i * 2];
		if (cur > maxCur) {
			maxCur = cur;
		}
		if (cur < minCur) {
			minCur = cur;
		}
		avgVol += vol;
		avgCur += cur;
	}

	avgVol /= BufferSize/4;
	avgCur /= BufferSize/4;

	constexpr uint16_t CurrentZeroNoise = 10;
	if(maxCur - minCur < CurrentZeroNoise) {
		// current was pretty much static for the whole PWM period -> probably no PWM active
		// -> adjust zero current offset
		currentZero = avgCur;
	}

	// remove offset from current values
	avgCur -= currentZero;

	// transform into SI values
	const uint16_t mV_ADC_Vol = avgVol * Defines::ADC_Ref / Defines::ADC_max;
	const int16_t mV_ADC_Cur = -avgCur * Defines::ADC_Ref / Defines::ADC_max;
	voltage = mV_ADC_Vol
			* (Defines::voltageDividerRHigh + Defines::voltageDividerRLow)
			/ Defines::voltageDividerRLow;
	current = (int64_t) mV_ADC_Cur * 1000000UL
			/ (Defines::currentShunt_uR * Defines::currentGain);

	voltage_sum += voltage;
	current_sum += current;
	samples++;
}

void HAL::BLDC::PowerADC::Init() {
	voltageInLimits = true;
	SetMaximumVoltageThreshold(16000);
	HAL_OPAMP_Start(&hopamp2);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) buf, BufferSize);
	HAL_TIM_Base_Start(&htim15);
	Log::Uart(Log::Lvl::Inf, "Initialized power ADC");
}

void HAL::BLDC::PowerADC::DMAComplete() {
	validBuf = &buf[BufferSize / 2];
	CalcVolCur();
}

void HAL::BLDC::PowerADC::DMAHalfComplete() {
	validBuf = &buf[0];
	CalcVolCur();
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


