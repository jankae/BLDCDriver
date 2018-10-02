#include "Communication.hpp"

#include "Logging.hpp"
#include "Propeller.hpp"
#include "Persistance.hpp"

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
		uint32_t recLength;
		xTaskNotifyWait(0, 0xffffffffUL, &recLength, portMAX_DELAY);

		// copy line into local buffer
		uint8_t line[recLength + 1];
		for(uint16_t i = 0;i<recLength;i++) {
			c->fifo.dequeue(line[i]);
		}
		// terminate string
		line[recLength] = 0;

		if(state == State::PropData && recLength < 10) {
			state = State::WaitForRPM;
		}

		switch (state) {
		case State::WaitForRPM:
			rpm = atol(line);
			if (rpm > 1000 && rpm < 8000) {
				state = State::FirstLine;
			} else if(strncmp(line, "save", 4) == 0) {
				Persistance::Store();
			} else if(strncmp(line, "clear", 5) == 0) {
				if (c->system->prop) {
					c->system->prop->ClearData();
				}
			} else if(strncmp(line, "print", 5) == 0) {
				if (c->system->prop) {
					c->system->prop->PrintData();
				}
			}
			break;
		case State::FirstLine:
			state = State::PropData;
			break;
		case State::PropData:
			if(strncmp(line, "ENDOFFILE", 9) == 0) {
				state = State::WaitForRPM;
			} else {
				if (c->system->prop) {
					c->system->prop->ParseDataLine(rpm, line);
				}
			}
			break;
		}
	}
}

Core::Communication::Communication() {
	system = nullptr;
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
		xTaskGenericNotifyFromISR(task, fifo.getLevel(), eSetValueWithOverwrite,
				nullptr, nullptr);
	}
}
