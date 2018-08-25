#pragma once

#include "BLDCHAL.hpp"

namespace HAL {
namespace BLDC {
class Driver : public HALDriver {
	Driver();

	void SetPWM(uint16_t promille) override;

	void InitiateStart() override;

	void RegisterIncCallback(IncCallback c, void *ptr) override;
	void RegisterADCCallback(ADCCallback c, void *ptr) override;
};
}
}
