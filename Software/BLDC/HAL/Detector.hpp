#pragma once

#include <cstdint>

namespace Core {

namespace BLDC {

namespace Detector {

using Callback = void(*)(uint32_t usSinceLast, uint32_t timeSinceCrossing);

void Init(Callback cb);
void Enable(uint8_t phase);
void Disable();

void DMAComplete();
void DMAHalfComplete();

}

}

}
