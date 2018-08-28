#include "Detector.hpp"

#include "stm32f3xx_hal.h"
#include "Logging.hpp"
#include <string.h>

#include "fifo.hpp"

//static Fifo<uint16_t, 3*700> buffer;// __attribute__ ((section (".ccmram")));;

static constexpr uint32_t ADC_Channels[] = { ADC_CHANNEL_1, ADC_CHANNEL_2,
		ADC_CHANNEL_3 };

static constexpr int ADCSampleRate = 20000;
static constexpr int ADCInspectionTimeus = 50;
static constexpr int ADCBufferLength = 3 * ADCSampleRate * 2 * ADCInspectionTimeus / 1000000UL;
static constexpr int PhaseSamplesPerInspection = ADCBufferLength / 6;

static_assert(ADCBufferLength % 6 == 0, "ADC Buffer length must be a multiple of 6");

static uint16_t ADCBuf[ADCBufferLength];
static uint16_t *ValidBuf = ADCBuf;

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3, htim1;

static uint8_t sensingPhase;
uint32_t timeUS;
static uint32_t lastCrossing;
static bool sensingActive;
static uint32_t SkipSamples;
static HAL::BLDC::Detector::Callback callback;
static uint32_t enableTime;
static bool DetectRising;
static uint16_t DetectionHysteresis;

static uint32_t crossingTime;
static bool crossingDetected;
static bool HysteresisValid;
static uint32_t HysteresisValidTime;

void HAL::BLDC::Detector::Init() {
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
	TIM1->CCR4 = 112;
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
//	TIM3->CNT = TIM3->ARR - 112;
	lastCrossing = 0;
	timeUS = 0;
	sensingActive = false;
	callback = nullptr;
}

void HAL::BLDC::Detector::Enable(Callback cb, uint16_t hyst) {
	SkipSamples = 2;
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

static constexpr uint64_t LinRegDenomConstSampling(uint16_t nvalues) {
	uint64_t ret = 0;
	for (uint16_t i = 0; i < nvalues; i++) {
		ret += (i - nvalues / 2) * (i - nvalues / 2);
	}
	return ret;
}


static void Analyze(uint16_t *data) {
	ValidBuf = data;
//	if(buffer.getSpace() < 4) {
//		uint16_t dummy;
//		buffer.dequeue(dummy);
//		buffer.dequeue(dummy);
//		buffer.dequeue(dummy);
//		buffer.dequeue(dummy);
//	}
//
//	buffer.enqueue(data[0]);
//	buffer.enqueue(data[1]);
//	buffer.enqueue(data[2]);
//	buffer.enqueue(3000 + sensingPhase * 300);
//
	if (!sensingActive) {
		return;
	}

	uint16_t compare = data[sensingPhase];
	uint16_t threshold = (data[1] + data[0] + data[2]) / 3;

	if (SkipSamples) {
		SkipSamples--;
		return;
	}

	if (!HysteresisValid
			&& (((compare > threshold + DetectionHysteresis) && !DetectRising)
					|| ((compare < threshold - DetectionHysteresis)
							&& DetectRising))) {
		HysteresisValid = true;
		HysteresisValidTime = timeUS;
		Log::Uart(Log::Lvl::Inf, "Hysteresis valid");
	}

	if (HysteresisValid && !crossingDetected
			&& (((compare < threshold) && !DetectRising)
					|| ((compare > threshold) && DetectRising))) {
		// zero crossing detected
		crossingTime = timeUS;
		crossingDetected = true;
	}

	if (HysteresisValid
			&& (((compare < threshold - DetectionHysteresis) && !DetectRising)
					|| ((compare > threshold + DetectionHysteresis)
							&& DetectRising))) {
		// Hysteresis crossed
		uint32_t timeSinceLast = crossingTime - lastCrossing;
		lastCrossing = crossingTime;
		uint32_t timeSinceCrossing = timeUS - crossingTime;
		if (callback) {
			callback(timeSinceLast, timeSinceCrossing);
		}

		if(DetectionHysteresis > 0) {
		Log::Uart(Log::Lvl::Inf,
				"Crossing, Hyst %d, (%lu/%lu/%lu)",
				DetectionHysteresis, HysteresisValidTime - enableTime, crossingTime - enableTime, timeUS - enableTime);
		}
	}
}

void HAL::BLDC::Detector::DMAComplete() {
	Analyze(&ADCBuf[ADCBufferLength / 2]);
	timeUS += ADCInspectionTimeus;
}

void HAL::BLDC::Detector::SetPhase(Phase p, bool rising) {
	sensingPhase = (uint8_t) p;
	DetectRising = !rising;
}

void HAL::BLDC::Detector::DMAHalfComplete() {
	Analyze(&ADCBuf[0]);
	timeUS += ADCInspectionTimeus;
}

uint16_t HAL::BLDC::Detector::GetLastSample(Phase p) {
	return ValidBuf[(int) p];
}

void HAL::BLDC::Detector::PrintBuffer() {
//	HAL_ADC_Stop_DMA(&hadc1);
//	while(buffer.getLevel() >= 3) {
//		uint16_t A, B, C, P;
//		buffer.dequeue(A);
//		buffer.dequeue(B);
//		buffer.dequeue(C);
//		buffer.dequeue(P);
//		Log::Uart(Log::Lvl::Inf, " %d;%d;%d;%d", A, B, C, P);
//		HAL_Delay(10);
//	}
}
