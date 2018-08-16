#pragma once

#include <cstdint>

namespace BLDC {

namespace HAL {

void SetStep(uint8_t step, uint16_t pwm);
void SetIdle();

}

}
