#pragma once

#include "BLDCHAL.hpp"

namespace HAL {
namespace BLDC {
class Driver : public HALDriver {
public:
	Driver();

	void SetPWM(int16_t promille) override;

	void FreeRunning();
	void Stop();


	void InitiateStart() override;

	void RegisterIncCallback(IncCallback c, void *ptr) override;
	void RegisterADCCallback(ADCCallback c, void *ptr) override;
};
}
}
