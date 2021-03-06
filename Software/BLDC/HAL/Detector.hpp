#pragma once

#include <cstdint>

namespace HAL {

namespace BLDC {

namespace Detector {

using Callback = void(*)(uint32_t usSinceLast, uint32_t timeSinceCrossing);
using IdleCallback = void(*)(uint8_t currentStep, bool valid);

enum class Phase : uint8_t {
	A = 0,
	B = 1,
	C = 2,
};

void Init();
void SetPhase(Phase p, bool rising);
void Enable(Callback cb, uint16_t hyst = 0);
void Disable();

void EnableIdleTracking(IdleCallback cb);
void DisableIdleTracking();

uint16_t GetLastSample(Phase p);

void DMAComplete();
void DMAHalfComplete();

bool isEnabled();
void Sample();
bool isSampling();

void PrintBuffer();

}

}

}
