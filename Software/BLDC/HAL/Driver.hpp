#pragma once

#include "BLDCHAL.hpp"

namespace HAL {
namespace BLDC {
class Driver : public HALDriver {
public:
	Driver();

	enum class State : uint8_t {
		Stopped,
		Starting,
		Running,
		Stopping,
	};

	void SetPWM(int16_t promille) override;

	void FreeRunning();
	void Stop();

	State GetState();


	void InitiateStart() override;

	void RegisterIncCallback(IncCallback c, void *ptr) override;
	void RegisterADCCallback(ADCCallback c, void *ptr) override;

};
}
}
