#include "lowlevel.hpp"

#include "stm32f3xx_hal.h"
#include "stm32f303x8.h"

static constexpr uint32_t Channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static constexpr uint16_t Pins[] = {PHASE_A_Pin, PHASE_B_Pin, PHASE_C_Pin};
static constexpr GPIO_TypeDef *Ports[] = {PHASE_A_GPIO_Port, PHASE_B_GPIO_Port, PHASE_C_GPIO_Port};

extern TIM_HandleTypeDef htim1;

static uint16_t pwmVal;

void HAL::BLDC::LowLevel::SetPWM(uint16_t pwm) {
	// directly modify PWM registers without HAL overhead
	pwmVal = pwm;
	TIM1->CCR1 = pwm;
	TIM1->CCR2 = pwm;
	TIM1->CCR3 = pwm;
}

//void BLDC::HAL::SetStep(uint8_t step, uint16_t pwm) {
//	if(step < 1 || step > 6) {
//		return;
//	}
//
//	uint32_t sensePin = 0;
//	GPIO_TypeDef *sensePort = GPIOA;
//	uint32_t posPWM = 0, negPWM = 0;
//
//	switch(step) {
//	case 1:
//		posPWM = PHASE_A_CHANNEL;
//		negPWM = PHASE_C_CHANNEL;
//		sensePin = PHASE_B_Pin;
//		sensePort = PHASE_B_GPIO_Port;
//		break;
//	case 2:
//		posPWM = PHASE_B_CHANNEL;
//		negPWM = PHASE_C_CHANNEL;
//		sensePin = PHASE_A_Pin;
//		sensePort = PHASE_A_GPIO_Port;
//		break;
//	case 3:
//		posPWM = PHASE_B_CHANNEL;
//		negPWM = PHASE_A_CHANNEL;
//		sensePin = PHASE_C_Pin;
//		sensePort = PHASE_C_GPIO_Port;
//		break;
//	case 4:
//		posPWM = PHASE_C_CHANNEL;
//		negPWM = PHASE_A_CHANNEL;
//		sensePin = PHASE_B_Pin;
//		sensePort = PHASE_B_GPIO_Port;
//		break;
//	case 5:
//		posPWM = PHASE_C_CHANNEL;
//		negPWM = PHASE_B_CHANNEL;
//		sensePin = PHASE_A_Pin;
//		sensePort = PHASE_A_GPIO_Port;
//		break;
//	case 6:
//		posPWM = PHASE_A_CHANNEL;
//		negPWM = PHASE_B_CHANNEL;
//		sensePin = PHASE_C_Pin;
//		sensePort = PHASE_C_GPIO_Port;
//		break;
//	}
//
//	// Set sense phase idle
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Pin = sensePin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(sensePort, &GPIO_InitStruct);
//
//	// Configure PWMs
//	TIM_OC_InitTypeDef sConfigOC;
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	sConfigOC.Pulse = pwm;
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, posPWM)
//			!= HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
//	}
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, negPWM)
//			!= HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
//	}
//
//	// enable PWM outputs
//    GPIO_InitStruct.Pin = (PHASE_A_Pin|PHASE_B_Pin|PHASE_C_Pin) & ~sensePin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//}

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
		break;
	case State::Idle:
		GPIO_InitStruct.Pin = Pins[(int) p];
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(Ports[(int) p], &GPIO_InitStruct);
	}
}
