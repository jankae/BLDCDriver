#pragma once

#include <cstdint>

namespace HAL {

namespace BLDC {

namespace Detector {

using Callback = void(*)(uint32_t usSinceLast, uint32_t timeSinceCrossing);

enum class Phase : uint8_t {
	A = 0,
	B = 1,
	C = 2,
};

void Init();
void SetPhase(Phase p, bool rising);
void Enable(Callback cb, uint16_t hyst = 0);
void Disable();

void DMAComplete();
void DMAHalfComplete();

void PrintBuffer();

}

}

}
