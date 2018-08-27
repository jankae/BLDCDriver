#include "Detector.hpp"

#include "stm32f3xx_hal.h"
#include "Logging.hpp"
#include <string.h>

#include "fifo.hpp"

static Fifo<uint16_t, 3*700> buffer;// __attribute__ ((section (".ccmram")));;

static constexpr uint32_t ADC_Channels[] = { ADC_CHANNEL_1, ADC_CHANNEL_2,
		ADC_CHANNEL_3 };

static constexpr int ADCSampleRate = 20000;
static constexpr int ADCInspectionTimeus = 50;
static constexpr int ADCBufferLength = 3 * ADCSampleRate * 2 * ADCInspectionTimeus / 1000000UL;
static constexpr int PhaseSamplesPerInspection = ADCBufferLength / 6;

static_assert(ADCBufferLength % 6 == 0, "ADC Buffer length must be a multiple of 6");

static uint16_t ADCBuf[ADCBufferLength];

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

static uint8_t sensingPhase;
uint32_t timeUS;
static uint32_t lastCrossing;
static bool sensingActive;
static uint32_t SkipSamples;
static HAL::BLDC::Detector::Callback callback;
static uint32_t enableTime;
static uint8_t selectedChannel;
static bool DetectRising;
static uint16_t DetectionHysteresis;

void write(const char *start, const char *end);

void HAL::BLDC::Detector::Init() {
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
	HAL_TIM_Base_Start(&htim3);
	TIM3->CNT = TIM3->ARR - 112;
	lastCrossing = 0;
	timeUS = 0;
	selectedChannel = 3;
	sensingActive = false;
	callback = nullptr;
}

void HAL::BLDC::Detector::Enable(Callback cb, uint16_t hyst) {
	sensingActive = true;
	SkipSamples = 2;
	callback = cb;
	enableTime = timeUS;
	DetectionHysteresis = hyst;
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

	if(buffer.getSpace() < 4) {
		uint16_t dummy;
		buffer.dequeue(dummy);
		buffer.dequeue(dummy);
		buffer.dequeue(dummy);
		buffer.dequeue(dummy);
	}

	buffer.enqueue(data[0]);
	buffer.enqueue(data[1]);
	buffer.enqueue(data[2]);
	buffer.enqueue(3000 + sensingPhase * 300);


	if (!sensingActive) {
		return;
	}
//	HAL_GPIO_TogglePin(TRIGGER_GPIO_Port, TRIGGER_Pin);
//	if (selectedChannel != sensingPhase) {
//		// Sensing phase changed, adjust ADC setting
//		HAL_ADC_Stop_DMA(&hadc1);
//		ADC_ChannelConfTypeDef sConfig;
//		/**Configure Regular Channel
//		 */
//		sConfig.Channel = ADC_Channels[sensingPhase];
//		sConfig.Rank = 1;
//		sConfig.SingleDiff = ADC_SINGLE_ENDED;
//		sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
//		sConfig.OffsetNumber = ADC_OFFSET_NONE;
//		sConfig.Offset = 0;
//		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
//			_Error_Handler(__FILE__, __LINE__);
//		}
//
//		/**Configure Regular Channel
//		 */
//		sConfig.Rank = 2;
//		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
//			_Error_Handler(__FILE__, __LINE__);
//		}
//
//		/**Configure Regular Channel
//		 */
//		sConfig.Rank = 3;
//		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
//			_Error_Handler(__FILE__, __LINE__);
//		}
//		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCBuf, ADCBufferLength);
//		selectedChannel = sensingPhase;
//		return;
//	}

	uint16_t databuf[3];
//	databuf[0] = data[0];
//	databuf[1] = data[1];
//	databuf[2] = data[2];

//	databuf[1] = (data[0] + data[1] + data[2]) / 3;

	databuf[1] = data[sensingPhase];
//	uint16_t max = 0;
//	for (uint8_t i = 0; i < 3; i++) {
//		if (data[i] > max)
//			max = data[i];
//	}
//	databuf[2] = max;

//	// bubble sort
//	if (databuf[0] > databuf[1]) {
//		uint16_t buf = databuf[0];
//		databuf[0] = databuf[1];
//		databuf[1] = buf;
//	}
//	if (databuf[1] > databuf[2]) {
//		uint16_t buf = databuf[1];
//		databuf[1] = databuf[2];
//		databuf[2] = buf;
//	}
//	if (databuf[0] > databuf[1]) {
//		uint16_t buf = databuf[0];
//		databuf[0] = databuf[1];
//		databuf[1] = buf;
//	}
//	char out[7];
//	out[0] = databuf[1] / 1000 + '0';
//	out[1] = (databuf[1] / 100)%10 + '0';
//	out[2] = (databuf[1] / 10)%10 + '0';
//	out[3] = (databuf[1] / 1)%10 + '0';
//	out[4] = sensingPhase + '0';
//	out[5] = '\n';
//	write(out, out+6);
//	Log::Uart(Log::Lvl::Inf, " %d", databuf[1]);


	uint16_t threshold = (data[1] + data[0] + data[2]) / 3;

//
//	static bool rising = false;
//	static bool falling = false;

	if (SkipSamples) {
		SkipSamples--;
//		if (!SkipSamples) {
//			if (databuf[1] < threshold - 200) {
////				Log::Uart(Log::Lvl::Inf, "Rising, (%d/%d/%d)", databuf[0],
////						databuf[1], databuf[2]);
//				rising = true;
//				falling = false;
//			} else if (databuf[1] > threshold + 200) {
//				falling = true;
//				rising = false;
////				Log::Uart(Log::Lvl::Inf, "Falling, (%d/%d/%d)", databuf[0],
////						databuf[1], databuf[2]);
//			} else {
//				if (rising ^ falling) {
//					falling = !falling;
//					rising = !rising;
//				}
////				Log::Uart(Log::Lvl::Inf, "Inconclusive, (%d/%d/%d)", databuf[0],
////						databuf[1], databuf[2]);
//			}
//		}
		return;
	}

	if (((databuf[1] < threshold - DetectionHysteresis) && !DetectRising)
			|| ((databuf[1] > threshold + DetectionHysteresis) && DetectRising)) {
		// zero crossing detected
		uint32_t crossingTime = timeUS;
		uint32_t timeSinceLast = crossingTime - lastCrossing;
		lastCrossing = crossingTime;
		uint32_t timeSinceCrossing = timeUS - enableTime;
		if (callback) {
			callback(timeSinceLast, timeSinceCrossing);
		}

//		Log::Uart(Log::Lvl::Inf,
//				"Crossing, rising: %d, (%d/%d/%d) at %lu, dt: %lu",
//				(int) rising, databuf[0], databuf[1], databuf[2], crossingTime, timeSinceLast);
	}


//	uint32_t avgSum[3];
//	avgSum[0] = avgSum[1] = avgSum[2] = 0;
//	for (uint16_t i = 0; i < ADCBufferLength / 2; i += 3) {
//		avgSum[0] += data[i];
//		avgSum[1] += data[i + 1];
//		avgSum[2] += data[i + 2];
//	}
//	avgSum[0] /= PhaseSamplesPerInspection;
//	avgSum[1] /= PhaseSamplesPerInspection;
//	avgSum[2] /= PhaseSamplesPerInspection;
//
//	// get average of driven phases (common zero point)
//	uint16_t ZeroPoint = (avgSum[0] + avgSum[1] + avgSum[2]
//			- avgSum[sensingPhase]) / 2;
//
//	// calculate linear regression of sensing phase
//	/*
//	 *  Part 1: get m value:
//	 *  m = sum_i_n(x_i - x_avg)*(y_i - y_avg) / sum_i_n(x_i - x_avg)
//	 */
//	constexpr int16_t x_avg = PhaseSamplesPerInspection / 2;
//	constexpr auto denom = LinRegDenomConstSampling(PhaseSamplesPerInspection);
//
//	int64_t num = 0;
//	const int16_t y_avg = avgSum[sensingPhase];
//	for (int16_t i = 0; i < PhaseSamplesPerInspection; i++) {
//		const int16_t y = data[i * 3 + sensingPhase];
//		num += (i - x_avg) * (y - y_avg);
//	}
//
//	float m = (float) num / denom;
//
//	int32_t b = y_avg - m * x_avg;
//
//	// calculate intersection with zero point
//	int32_t intersect = (ZeroPoint - b) / m;
//	if (abs(intersect) <= PhaseSamplesPerInspection && (m >= 2 || m <= -2)) {
//		// intersection happens within the sampled data or is already over
//		uint32_t crossingTime = timeUS
//				+ intersect * ADCInspectionTimeus / PhaseSamplesPerInspection;
//		uint32_t timeSinceLast = crossingTime - lastCrossing;
//		lastCrossing = crossingTime;
//		uint32_t timeSinceCrossing = (PhaseSamplesPerInspection - intersect)
//				* ADCInspectionTimeus / PhaseSamplesPerInspection;
//		callback(timeSinceLast, timeSinceCrossing);
//
//		Log::Uart(Log::Lvl::Inf, "Crossing at %luus (%luus since last, detected at %luus, %luus ago), m: %f",
//				crossingTime, timeSinceLast, timeUS, timeSinceCrossing, m);
//	}
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

void HAL::BLDC::Detector::PrintBuffer() {
	HAL_ADC_Stop_DMA(&hadc1);
	while(buffer.getLevel() >= 3) {
		uint16_t A, B, C, P;
		buffer.dequeue(A);
		buffer.dequeue(B);
		buffer.dequeue(C);
		buffer.dequeue(P);
		Log::Uart(Log::Lvl::Inf, " %d;%d;%d;%d", A, B, C, P);
		HAL_Delay(10);
	}
}
