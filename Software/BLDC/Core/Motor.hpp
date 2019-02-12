#pragma once

#include "BLDCHAL.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Sysinfo.hpp"

#include <cstdint>

namespace Core {
class Motor {
public:
	using Coefficients = struct coefficients {
		float resistance;
		float inductance;
		float Kv;
	};

	enum class ControlMode : uint8_t {
			Off,
			Promille,
			RPM,
			Thrust,
	};

	using InState = struct {
		int32_t value;
		ControlMode mode;
	} __attribute__ ((packed));

	using OutState = struct {
		int32_t Current;
		int32_t Voltage;
		int32_t RPM;
		int32_t Thrust;
		int32_t Torque;
	} __attribute__ ((packed));

	Motor(Sysinfo &s);
	~Motor();

	void NewData();
	OutState outState;
	InState inState;
private:
	static constexpr uint32_t ControllerStack = 200;
	static constexpr uint8_t ControllerPriority = FreeRTOSPrioRealtime;
	static constexpr TickType_t ControllerPeriod = pdMS_TO_TICKS(1);

	void ControlTask();

	portTickType lastExecutionTime;

	xTaskHandle ControlHandle;
	Sysinfo *sys;
};
}

