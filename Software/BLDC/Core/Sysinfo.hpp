#pragma once

namespace Core {
// Forward declare used classes
class Communication;
class Propeller;

using Sysinfo = struct sysinfo {
	Propeller *prop;
	Communication *communication;
};

}
