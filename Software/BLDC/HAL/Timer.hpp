#pragma once

#include <cstdint>

namespace HAL {
namespace BLDC {
namespace Timer {

using Callback = void(*)(void);

void Schedule(uint32_t usTillExecution, Callback ptr);
void Abort(void);

}
}
}
