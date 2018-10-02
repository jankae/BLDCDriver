#pragma once

#include <cstdint>
#include "fifo.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "Sysinfo.hpp"

namespace Core {
class Communication {
public:
	Communication();

	void SetSystemInfo(Sysinfo *s);

	void NewData(uint8_t d);
private:
	static void Task(void *argument);
	static constexpr uint16_t MaxBufferSize = 250;
	static constexpr uint8_t endDelimiter = '\n';

	Fifo<uint8_t, MaxBufferSize> fifo;
	uint16_t lineEndings;
	TaskHandle_t task;

	Sysinfo *system;
};
}
