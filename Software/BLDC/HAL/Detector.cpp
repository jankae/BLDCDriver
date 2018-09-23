#include "Detector.hpp"

#include "stm32f3xx_hal.h"
#include "Logging.hpp"
#include <string.h>
#include "lowlevel.hpp"

#include "fifo.hpp"

using namespace HAL::BLDC;

static Fifo<uint16_t, 1500> buffer __attribute__ ((section (".ccmram")));;

static constexpr int ADCInspectionTimeus = 50;
static constexpr int ADCBufferLength = 6;

static uint16_t ADCBuf[ADCBufferLength];
static uint16_t *ValidBuf = ADCBuf;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

static uint8_t sensingPhase;
uint32_t timeUS;
static uint32_t lastCrossing;
static bool sensingActive;
static uint32_t SkipSamples;
static uint32_t crossingTime;
static bool crossingDetected;
static bool HysteresisValid;
static uint32_t HysteresisValidTime;
static HAL::BLDC::Detector::Callback callback;
static uint32_t enableTime;
static bool DetectRising;
static uint16_t DetectionHysteresis;

static bool sampling;

static bool idleTracking;
static bool skipNextIdleSample;
static HAL::BLDC::Detector::IdleCallback idleCallback;
static constexpr uint16_t idleDetectionThreshold = 25;
static constexpr uint16_t idleDetectionHysterese = 15;


void HAL::BLDC::Detector::Init() {
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
	TIM1->CCR4 = 112;
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
	lastCrossing = 0;
	timeUS = 0;
	sensingActive = false;
	callback = nullptr;
	buffer.clear();
}

void HAL::BLDC::Detector::Enable(Callback cb, uint16_t hyst) {
	SkipSamples = 3;
	callback = cb;
	enableTime = timeUS;
	DetectionHysteresis = hyst;
	crossingDetected = false;
	if(DetectionHysteresis>0) {
		HysteresisValid = false;
	} else {
		HysteresisValid = true;
	}
	sensingActive = true;
}

void HAL::BLDC::Detector::Disable() {
	sensingActive = false;
}

static void Analyze(uint16_t *data) {
	ValidBuf = data;

	if(sampling) {
		if (buffer.getSpace()) {
			buffer.enqueue(data[0]);
			buffer.enqueue(data[1]);
			buffer.enqueue(data[2]);
		} else {
			sampling = false;
		}
	}

	if (sensingActive) {
		Log::WriteChar('A');

		uint16_t compare = data[sensingPhase];
		uint16_t threshold = (data[1] + data[0] + data[2]) / 3;

		if (SkipSamples) {
			SkipSamples--;
			Log::WriteChar('S');
			return;
		}

		if (!HysteresisValid
				&& (((compare > threshold + DetectionHysteresis)
						&& !DetectRising)
						|| ((compare < threshold - DetectionHysteresis)
								&& DetectRising))) {
			HysteresisValid = true;
			HysteresisValidTime = timeUS;
			Log::Uart(Log::Lvl::Inf, "Hysteresis valid");
			Log::WriteChar('H');
		}

		if (HysteresisValid && !crossingDetected
				&& (((compare < threshold) && !DetectRising)
						|| ((compare > threshold) && DetectRising))) {
			// zero crossing detected
			crossingTime = timeUS;
			crossingDetected = true;
			Log::WriteChar('C');
		}

		if (HysteresisValid
				&& (((compare < threshold - DetectionHysteresis)
						&& !DetectRising)
						|| ((compare > threshold + DetectionHysteresis)
								&& DetectRising))) {
			// Hysteresis crossed
			uint32_t timeSinceLast = crossingTime - lastCrossing;
			lastCrossing = crossingTime;
			uint32_t timeSinceCrossing = timeUS - crossingTime;
			Log::WriteChar('D');
			if (callback) {
				callback(timeSinceLast, timeSinceCrossing);
			}

			if (DetectionHysteresis > 0) {
				Log::Uart(Log::Lvl::Inf, "Crossing, Hyst %d, (%lu/%lu/%lu)",
						DetectionHysteresis, HysteresisValidTime - enableTime,
						crossingTime - enableTime, timeUS - enableTime);
			}
		}
	}

	if(idleTracking) {
		if(skipNextIdleSample) {
			skipNextIdleSample = false;
		} else {
			uint16_t max = data[0];
			if (data[1] > max) {
				max = data[1];
			}
			if (data[2] > max) {
				max = data[2];
			}
			static bool valid = false;
			uint8_t pos = 0;
			if (valid && max < idleDetectionThreshold - idleDetectionHysterese) {
				valid = false;
			} else if(!valid && max > idleDetectionThreshold + idleDetectionHysterese) {
				valid = true;
			}
			const auto A = data[(int) Detector::Phase::A];
			const auto B = data[(int) Detector::Phase::B];
			const auto C = data[(int) Detector::Phase::C];
			if (A > B && B >= C) {
				pos = 0;
			} else if (B > A && A >= C) {
				pos = 1;
			} else if (B > C && C >= A) {
				pos = 2;
			} else if (C > B && B >= A) {
				pos = 3;
			} else if (C > A && A >= B) {
				pos = 4;
			} else if (A > C && C >= B) {
				pos = 5;
			}
			lastCrossing = timeUS;
			if(idleCallback) {
				idleCallback(pos, valid);
			}
		}
	}
}

void HAL::BLDC::Detector::DMAComplete() {
	Analyze(&ADCBuf[ADCBufferLength / 2]);
	timeUS += ADCInspectionTimeus;
}

void HAL::BLDC::Detector::SetPhase(Phase p, bool rising) {
	sensingPhase = (uint8_t) p;
	DetectRising = rising;
}

void HAL::BLDC::Detector::DMAHalfComplete() {
	Analyze(&ADCBuf[0]);
	timeUS += ADCInspectionTimeus;
}

uint16_t HAL::BLDC::Detector::GetLastSample(Phase p) {
	return ValidBuf[(int) p];
}

bool HAL::BLDC::Detector::isEnabled() {
	return sensingActive;
}

void HAL::BLDC::Detector::Sample() {
	sampling = true;
}

bool HAL::BLDC::Detector::isSampling() {
	return sampling;
}

void HAL::BLDC::Detector::EnableIdleTracking(IdleCallback cb) {
	skipNextIdleSample = true;
	idleTracking = true;
	idleCallback = cb;
}

void HAL::BLDC::Detector::DisableIdleTracking() {
	idleTracking = false;
}

void HAL::BLDC::Detector::PrintBuffer() {
	while(buffer.getLevel() >= 3) {
		uint16_t A = 0, B = 0, C = 0;
		buffer.dequeue(A);
		buffer.dequeue(B);
		buffer.dequeue(C);
		Log::Uart(Log::Lvl::Inf, " %d;%d;%d", A, B, C);
		HAL_Delay(10);
	}
	buffer.clear();
}
