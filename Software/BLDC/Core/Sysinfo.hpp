#pragma once

namespace HAL {
namespace BLDC {
class Driver;
}
}

namespace Core {
// Forward declare used classes
class Communication;
class Propeller;

using Sysinfo = struct sysinfo {
	Propeller *prop;
	Communication *communication;
	HAL::BLDC::Driver *driver;
};

}
