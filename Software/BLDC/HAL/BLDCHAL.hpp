#pragma once

#include <cstdint>

namespace HAL {

namespace BLDC {

class HAL {
public:
	using IncCallback = void (*)(void* ptr, uint32_t usSinceLast);
	using ADCCallback = void (*)(void* ptr, uint32_t voltage, uint32_t current);

	virtual void SetPWM(uint16_t promille) = 0;

	virtual void InitiateStart() = 0;

	virtual void RegisterIncCallback(IncCallback c, void *ptr) = 0;
	virtual void RegisterADCCallback(ADCCallback c, void *ptr) = 0;
};

}

}
