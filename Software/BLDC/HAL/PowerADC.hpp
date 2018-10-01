#pragma once

#include <cstdint>

namespace HAL {
namespace BLDC {
namespace PowerADC {

using Measurement = struct meas {
	uint32_t voltage; // in mV
	int32_t current; // in mA
};

void Init();
void Pause();
void Resume();

void SetMaximumVoltageThreshold(uint16_t mV);
bool VoltageWithinLimits();

void DMAComplete();
void DMAHalfComplete();

void PrintBuffer();

Measurement Get();

}
}
}
