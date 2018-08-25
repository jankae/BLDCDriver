#include "lowlevel.hpp"

#include "stm32f3xx_hal.h"
#include "stm32f303x8.h"

static constexpr uint32_t Channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static constexpr uint16_t Pins[] = {PHASE_A_Pin, PHASE_B_Pin, PHASE_C_Pin};
static constexpr GPIO_TypeDef *Ports[] = {PHASE_A_GPIO_Port, PHASE_B_GPIO_Port, PHASE_C_GPIO_Port};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

static uint16_t pwmVal;

void HAL::BLDC::LowLevel::Init() {
	__HAL_DBGMCU_FREEZE_TIM2();
	__HAL_DBGMCU_FREEZE_TIM6();

	SetPhase(Phase::A, State::Idle);
	SetPhase(Phase::B, State::Idle);
	SetPhase(Phase::C, State::Idle);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);
}

void HAL::BLDC::LowLevel::SetPWM(uint16_t pwm) {
	// directly modify PWM registers without HAL overhead
	pwmVal = pwm;
	TIM1->CCR1 = pwm;
	TIM1->CCR2 = pwm;
	TIM1->CCR3 = pwm;
}

void HAL::BLDC::LowLevel::SetPhase(Phase p, State s) {
	TIM_OC_InitTypeDef sConfigOC;
	GPIO_InitTypeDef GPIO_InitStruct;

	switch(s) {
	case State::High:
		// Configure PWMs
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = pwmVal;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, Channels[(int) p])
				!= HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}
		// enable PWM outputs
	    GPIO_InitStruct.Pin = Pins[(int) p];
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
	    HAL_GPIO_Init(Ports[(int) p], &GPIO_InitStruct);
		HAL_TIM_PWM_Start(&htim1, Channels[(int) p]);
		break;
	case State::Low:
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = pwmVal;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, Channels[(int) p])
				!= HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}
		// enable PWM outputs
	    GPIO_InitStruct.Pin = Pins[(int) p];
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
	    HAL_GPIO_Init(Ports[(int) p], &GPIO_InitStruct);
		HAL_TIM_PWM_Start(&htim1, Channels[(int) p]);
		break;
	case State::Idle:
		GPIO_InitStruct.Pin = Pins[(int) p];
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(Ports[(int) p], &GPIO_InitStruct);
		break;
	}
}

#include "Detector.hpp"

extern "C" {
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC1) {
		HAL::BLDC::Detector::DMAComplete();
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC1) {
		HAL::BLDC::Detector::DMAHalfComplete();
	}
}
}

