#include "PowerADC.hpp"

#include "stm32f3xx_hal.h"

#include "Logging.hpp"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim15;
extern OPAMP_HandleTypeDef hopamp2;

static constexpr uint16_t BufferSize = 500;

uint16_t buf[BufferSize];

void Stop() {
	HAL_ADC_Stop_DMA(&hadc2);
}

void PrintBuf() {
	for(uint16_t i = 0;i<BufferSize;i++) {
		Log::Uart(Log::Lvl::Inf, "%d", buf[i]);
		HAL_Delay(1);
	}
}


void HAL::BLDC::PowerADC::Pause() {
	HAL_ADC_Stop_DMA(&hadc2);
}

void HAL::BLDC::PowerADC::Resume() {
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) buf, BufferSize);
}


void HAL::BLDC::PowerADC::Init() {
	HAL_OPAMP_Start(&hopamp2);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) buf, BufferSize);
	HAL_TIM_Base_Start(&htim15);
}

void HAL::BLDC::PowerADC::DMAComplete() {
}

void HAL::BLDC::PowerADC::DMAHalfComplete() {
}
