#include "Driver.hpp"

#include "lowlevel.hpp"

void Core::BLDC::Driver::SetPWM(uint16_t promille) {
	LowLevel::SetPWM(
			LowLevel::MaxPWM / 2
					+ (uint32_t) LowLevel::MaxPWM / 2 * promille / 1000);
}

void Core::BLDC::Driver::InitiateStart() {
}

void Core::BLDC::Driver::RegisterIncCallback(IncCallback c, void* ptr) {
}

void Core::BLDC::Driver::RegisterADCCallback(ADCCallback c, void* ptr) {
}
