#pragma once

#include <cstdint>

namespace Core {

namespace BLDC {

namespace LowLevel {

constexpr uint16_t MaxPWM = 1600;

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

void SetPWM(uint16_t pwm);
void SetPhase(Phase p, State s);

}

}

}
