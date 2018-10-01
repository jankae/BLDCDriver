#pragma once

#include <cstdint>

namespace HAL {
namespace BLDC {
namespace PowerADC {

void Init();
void Pause();
void Resume();

void SetMaximumVoltageThreshold(uint16_t mV);
bool VoltageWithinLimits();

void DMAComplete();
void DMAHalfComplete();

void PrintBuffer();

}
}
}
