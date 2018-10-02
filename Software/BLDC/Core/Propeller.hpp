#pragma once

#include <cstdint>

namespace Core {
class Propeller {
public:

static constexpr uint8_t MaxDiffRPMs = 4;
static constexpr uint8_t MaxDiffAdvance = 20;
static constexpr float rho = 1.2041;

static constexpr float alpha = 0.01;

using CoefficientEntry = struct CoeffEntry {
	float J, Ct, Cp, Eta;
};

using RPMTable = struct rpmtable {
	uint16_t rpm;
	uint16_t nentries;
	CoefficientEntry entries[MaxDiffAdvance];
};

using Data = struct data {
	float diameter, inertia;
	RPMTable rpm[MaxDiffRPMs];
	uint16_t nRPMs;
};

	Propeller(Data *data);

	void ClearData(void);
	bool ParseDataLine(uint16_t rpm, char *line);
	void PrintData(void);

	void Update(float rpm, float appliedPower);
	float ThrustToRPM(float thrust);
	float RPMToThrust(float rpm);
	float TorqueToRPM(float torque);
	float RPMToTorque(float rpm);

private:
	Data *d;

	float Ct, Cp, V;
	float Torque, Thrust;
	float D4, D5;

};

}
