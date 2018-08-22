#include "Detector.hpp"

#include "stm32f3xx_hal.h"

static constexpr int ADCSampleRate = 500000;
static constexpr int ADCInspectionTimeus = 504;
static constexpr int ADCBufferLength = ADCSampleRate * 2 * ADCInspectionTimeus / 1000000UL;
static constexpr int PhaseSamplesPerInspection = ADCBufferLength / 6;

static_assert(ADCBufferLength % 6 == 0, "ADC Buffer length must be a multiple of 6");

static uint16_t ADCBuf[ADCBufferLength];

extern ADC_HandleTypeDef hadc1;

static uint8_t sensingPhase;
static uint32_t timeUS;
static uint32_t lastCrossing;
static bool sensingActive;
static bool SkipNextInspectionWindow;
static HAL::BLDC::Detector::Callback callback;

void HAL::BLDC::Detector::Init(Callback cb) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
	lastCrossing = 0;
	timeUS = 0;
	sensingActive = false;
	callback = cb;
}

void HAL::BLDC::Detector::Enable(Phase phase) {
	sensingPhase = (uint8_t) phase;
	sensingActive = true;
	SkipNextInspectionWindow = true;
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
	if (!sensingActive) {
		return;
	}
	if (SkipNextInspectionWindow) {
		SkipNextInspectionWindow = false;
		return;
	}

	uint32_t avgSum[3];
	for (uint16_t i = 0; i < ADCBufferLength / 2; i += 3) {
		avgSum[0] += data[i];
		avgSum[1] += data[i + 1];
		avgSum[2] += data[i + 2];
	}
	avgSum[0] /= PhaseSamplesPerInspection;
	avgSum[1] /= PhaseSamplesPerInspection;
	avgSum[2] /= PhaseSamplesPerInspection;

	// get average of driven phases (common zero point)
	uint16_t ZeroPoint = (avgSum[0] + avgSum[1] + avgSum[2]
			- avgSum[sensingPhase]) / 2;

	// calculate linear regression of sensing phase
	/*
	 *  Part 1: get m value:
	 *  m = sum_i_n(x_i - x_avg)*(y_i - y_avg) / sum_i_n(x_i - x_avg)
	 */
	constexpr uint16_t x_avg = PhaseSamplesPerInspection / 2;
	constexpr auto denom = LinRegDenomConstSampling(PhaseSamplesPerInspection);

	uint64_t num = 0;
	const uint16_t y_avg = avgSum[sensingPhase];
	for (uint16_t i = 0; i < PhaseSamplesPerInspection; i++) {
		const uint16_t y = data[i * 3 + sensingPhase];
		num += (i - x_avg) * (y - y_avg);
	}

	float m = (float) num / denom;

	uint16_t b = y_avg - m * x_avg;

	// calculate intersection with zero point
	int16_t intersect = (ZeroPoint - b) / m;
	if (intersect <= PhaseSamplesPerInspection) {
		// intersection happens within the sampled data or is already over
		uint32_t crossingTime = timeUS
				+ intersect * ADCInspectionTimeus / PhaseSamplesPerInspection;
		uint32_t timeSinceLast = crossingTime - lastCrossing;
		lastCrossing = crossingTime;
		uint32_t timeSinceCrossing = (PhaseSamplesPerInspection - intersect)
				* ADCInspectionTimeus / PhaseSamplesPerInspection;
		callback(timeSinceLast, timeSinceCrossing);
	}
}

void HAL::BLDC::Detector::DMAComplete() {
	Analyze(&ADCBuf[ADCBufferLength / 2]);
	timeUS += ADCInspectionTimeus;
}


void HAL::BLDC::Detector::DMAHalfComplete() {
	Analyze(&ADCBuf[0]);
	timeUS += ADCInspectionTimeus;
}
