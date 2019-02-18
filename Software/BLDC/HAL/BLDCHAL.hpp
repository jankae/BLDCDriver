#pragma once

#include <cstdint>

namespace HAL {

namespace BLDC {

class HALDriver {
public:
	virtual ~HALDriver(){};

	using IncCallback = void (*)(void* ptr, uint32_t usSinceLast);
	using ADCCallback = void (*)(void* ptr, uint32_t voltage, uint32_t current);

	using MotorData = struct motordata {
		uint32_t current;
		uint32_t voltage;
		uint16_t rpm;
	};

	enum class TestResult : uint8_t {
		OK,
		NoMotor,
		Failure,
	};

	virtual void SetPWM(int16_t promille) = 0;

	virtual void InitiateStart() = 0;
	virtual void Stop() = 0;

	enum class State : uint8_t {
		Stopped,
		Starting,
		Running,
		Stopping,
	};

	State GetState() { return state; };

	virtual TestResult Test() = 0;

	virtual MotorData GetData() = 0;
protected:
	State state;
};

}

}
