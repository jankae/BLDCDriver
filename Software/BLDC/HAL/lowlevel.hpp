#pragma once

#include <cstdint>

namespace HAL {

namespace BLDC {

namespace LowLevel {

enum class Phase : uint8_t {
	A = 0,
	B = 1,
	C = 2,
};

enum class State : uint8_t {
	High,
	Low,
	Idle,
};

void Init();
void SetPWM(int16_t promille);
void SetPhase(Phase p, State s);

}

}

}
