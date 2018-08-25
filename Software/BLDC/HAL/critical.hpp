#pragma once

#include "stm32f3xx_hal.h"

class CriticalSection {
public:
	CriticalSection() {
		prim = __get_PRIMASK();
		__disable_irq();
	}
	~CriticalSection() {
		if(!prim)
			__enable_irq();
	}
private:
	uint32_t prim;
};
