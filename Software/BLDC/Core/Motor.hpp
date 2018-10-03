#pragma once

#include "BLDCHAL.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
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
			RPM,
			Thrust,
	};

	using Setpoint = struct {
		ControlMode mode;
		uint32_t value;
	};

	using Measurement = struct {
		uint32_t Thrust;
		uint32_t Torque;
		uint32_t RPM;
		uint32_t Voltage;
		uint32_t Current;
	};

	Motor(HAL &h);
	~Motor();

	void SetSystemInfo(Sysinfo *s);

	/*
	 * Functions called by the interface to the main copter control.
	 * Set desired motor variables and read the status back
	 */
	void Set(Setpoint setpoint);

	Measurement GetMeasurement() {
		return outState;
	}

private:
	static constexpr uint32_t ControllerStack = 200;
	static constexpr uint8_t ControllerPriority = FreeRTOSPrioRealtime;
	static constexpr TickType_t ControllerPeriod = pdMS_TO_TICKS(1);

	void ControlTask();

	xTaskHandle ControlHandle;
	HAL *hal;


	OutState outState;
	InState inState;

	uint32_t Pgain;

	portTickType lastStateTransition;
	portTickType lastControllerIteration;

	Sysinfo *system;
};
}

