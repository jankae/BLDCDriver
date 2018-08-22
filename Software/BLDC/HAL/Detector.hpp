#pragma once

#include <cstdint>

namespace Core {

namespace BLDC {

namespace Detector {

using Callback = void(*)(uint32_t usSinceLast, uint32_t timeSinceCrossing);

enum class Phase : uint8_t {
	A = 0,
	B = 1,
	C = 2,
};

void Init(Callback cb);
void Enable(Phase phase);
void Disable();

void DMAComplete();
void DMAHalfComplete();

}

}

}
