#pragma once

namespace HAL {
namespace BLDC {
namespace PowerADC {

void Init();
void Pause();
void Resume();

void DMAComplete();
void DMAHalfComplete();

}
}
}
