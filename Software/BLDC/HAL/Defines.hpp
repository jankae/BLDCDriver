#pragma once

#include <cstdint>

namespace HAL {
namespace BLDC {
namespace Defines {

/* Adjust to hardware */
constexpr uint16_t voltageDividerRHigh = 4700;
constexpr uint16_t voltageDividerRLow = 1000;

constexpr uint32_t currentShunt_uR = 1000;
constexpr uint16_t currentGain = 50;

/* ADC settings */
constexpr uint16_t ADC_Ref = 3300;
constexpr uint16_t ADC_max = 4096;
constexpr uint32_t ADC_SampleratePower = 1000000;

/* PWM Settings */
constexpr uint16_t PWM_Frequency = 20000;
constexpr uint16_t PWM_max = 1600;

}
}
}
