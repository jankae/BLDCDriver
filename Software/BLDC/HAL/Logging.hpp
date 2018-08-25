#pragma once

#include <string>

namespace Log {

enum class Lvl {
	Dbg = 0,
	Inf = 1,
	Wrn = 2,
	Err = 3,
	Crt = 4,
};

enum class Class {
	BLDC,
	// Max has to be last
	MAX
};

void Init(enum Lvl lvl);
void SetLevel(enum Class cls, enum Lvl lvl);

void Out(enum Class cls, enum Lvl lvl, const char *fmt, ...);

void Uart(enum Class cls, enum Lvl lvl, const char *fmt, ...);

}
