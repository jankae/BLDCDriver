#include "Communication.hpp"

#include "Logging.hpp"
#include "Propeller.hpp"
#include "Persistance.hpp"
#include "Driver.hpp"

#include <cstring>

using namespace Core;

static uint16_t rpm;
enum class State : uint8_t {
	WaitForRPM,
	FirstLine,
	PropData,
};

State state = State::WaitForRPM;

void Core::Communication::Task(void *argument) {
	Communication *c = (Communication*) argument;
	Log::Uart(Log::Lvl::Inf, "Communication task running");
	while(1) {
		xTaskNotifyWait(0, 0xffffffffUL, nullptr, portMAX_DELAY);
		uint16_t level = c->fifo.getLevel();

		while (c->lineEndings) {
			// copy line into local buffer
			uint8_t line[level];
			uint16_t i;
			for (i = 0; i < level; i++) {
				c->fifo.dequeue(line[i]);
				if (line[i] == '\n')
					break;
			}
			// terminate string
			line[i + 1] = 0;

			switch (state) {
			case State::WaitForRPM:
				rpm = atol(line);
				if (rpm > 1000 && rpm < 8000) {
					state = State::FirstLine;
				} else if (strncmp(line, "save", 4) == 0) {
					Persistance::Store();
				} else if (strncmp(line, "clear", 5) == 0) {
					if (c->system->prop) {
						c->system->prop->ClearData();
					}
				} else if (strncmp(line, "print", 5) == 0) {
					if (c->system->prop) {
						c->system->prop->PrintData();
					}
				} else if (strncmp(line, "calib", 5) == 0) {
					if (c->system->driver) {
//						c->system->driver->Calibrate();
					}
				}
				break;
			case State::FirstLine:
				state = State::PropData;
				break;
			case State::PropData:
				if (strncmp(line, "ENDOFFILE", 9) == 0) {
					state = State::WaitForRPM;
				} else {
					if (c->system->prop) {
						c->system->prop->ParseDataLine(rpm, line);
					}
				}
				break;
			}
			{
				CriticalSection crit;
				c->lineEndings--;
			}
		}
	}
}

Core::Communication::Communication() {
	system = nullptr;
	lineEndings = 0;
	if(!xTaskCreate(Task, "ComTask", 500, this, FreeRTOSPrioLow, &task)) {
		Log::Uart(Log::Lvl::Err, "Failed to create communication task");
		return;
	}
	Log::Uart(Log::Lvl::Inf, "Created communication task");
}

void Core::Communication::SetSystemInfo(Sysinfo* s) {
	system = s;
}

void Core::Communication::NewData(uint8_t d) {
	fifo.enqueue(d);
	if(d == endDelimiter) {
		lineEndings++;
		xTaskGenericNotifyFromISR(task, fifo.getLevel(), eSetValueWithOverwrite,
				nullptr, nullptr);
	}
}
