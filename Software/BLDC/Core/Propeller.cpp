#include "Propeller.hpp"

#include <math.h>
#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "Logging.hpp"

Core::Propeller::Propeller(Data *data) {
	d = data;
	// Todo: read real values
	d->diameter = 0.28f;
	d->inertia = 0.00004f;

	D4 = d->diameter * d->diameter * d->diameter * d->diameter;
	D5 = D4 * d->diameter;

	Log::Uart(Log::Lvl::Inf, "Created propeller object");

	// Check data for plausibility
	Log::Uart(Log::Lvl::Dbg, "Prop data: rpms: %d", d->nRPMs);
	bool plausible = true;
	if (d->nRPMs <= MaxDiffRPMs) {
		for (uint8_t i = 0; i < d->nRPMs; i++) {
			Log::Uart(Log::Lvl::Dbg, "Prop data: RPM: %d, entries: %d",
					d->rpm[i].rpm, d->rpm[i].nentries);
			if (d->rpm[i].nentries > MaxDiffAdvance) {
				plausible = false;
			}
		}
	} else {
		plausible = false;
	}

	if (!plausible) {
		Log::Uart(Log::Lvl::Wrn, "Implausible prop data, clear coefficients");
		ClearData();
	}
}

static float interpolate(float value, float from1, float from2, float to1, float to2) {
	float alpha = (value - from1) / (from2 - from1);
	return to1 + alpha * (to2 - to1);
}

static float InterpolateJFromCp(Core::Propeller::RPMTable &t, float Cp) {
	uint8_t i;
	for(i = 0;i<t.nentries;i++) {
		if(t.entries[i].Cp < Cp)
			break;
	}
	if (i == 0) {
		return t.entries[0].J;
	} else if (i == t.nentries) {
		return t.entries[i - 1].J;
	} else {
		return interpolate(Cp, t.entries[i - 1].Cp, t.entries[i].Cp,
				t.entries[i - 1].J, t.entries[i].J);
	}
}

static float InterpolateJ(Core::Propeller::Data &d, float Cp, float rpm) {
	uint8_t i;
	for(i = 0;i<d.nRPMs;i++) {
		if(d.rpm[i].rpm > rpm)
			break;
	}
	if (i == 0) {
		return InterpolateJFromCp(d.rpm[0], Cp);
	} else if (i == d.nRPMs) {
		return InterpolateJFromCp(d.rpm[i - 1], Cp);
	} else {
		return interpolate(rpm, d.rpm[i - 1].rpm, d.rpm[i].rpm,
				InterpolateJFromCp(d.rpm[i - 1], Cp),
				InterpolateJFromCp(d.rpm[i], Cp));
	}
}

static float InterpolateCtFromJ(Core::Propeller::RPMTable &t, float J) {
	uint8_t i;
	for(i = 0;i<t.nentries;i++) {
		if(t.entries[i].J > J)
			break;
	}
	if (i == 0) {
		return t.entries[0].Ct;
	} else if (i == t.nentries) {
		return t.entries[i - 1].Ct;
	} else {
		return interpolate(J, t.entries[i - 1].J, t.entries[i].J,
				t.entries[i - 1].Ct, t.entries[i].Ct);
	}
}

static float InterpolateCt(Core::Propeller::Data &d, float J, float rpm) {
	uint8_t i;
	for(i = 0;i<d.nRPMs;i++) {
		if(d.rpm[i].rpm > rpm)
			break;
	}
	if (i == 0) {
		return InterpolateCtFromJ(d.rpm[0], J);
	} else if (i == d.nRPMs) {
		return InterpolateCtFromJ(d.rpm[i - 1], J);
	} else {
		return interpolate(rpm, d.rpm[i - 1].rpm, d.rpm[i].rpm,
				InterpolateCtFromJ(d.rpm[i - 1], J),
				InterpolateCtFromJ(d.rpm[i], J));
	}
}

void Core::Propeller::Update(float rpm, float appliedPower) {
	if (rpm < 400) {
		// calculation is unreliable for low rpm, skip
		return;
	}
	const float n = rpm / 60;
	const float n2 = n * n;
	const float n3 = n2 * n;
	float cp = appliedPower / (rho * n3 * D5);
	Cp = cp * alpha + Cp * (1 - alpha);

	// update torque
	Torque = Cp * rho * n2 * D5 / (2 * 3.141f);

	J = InterpolateJ(*d, Cp, rpm);

	V = J * n * d->diameter;

	Ct = InterpolateCt(*d, J, rpm);

	// update thrust
	Thrust = Ct * rho * n2 * D4;
}

float Core::Propeller::ThrustToRPM(float thrust) {
	const float n2 = thrust / (rho * Ct * D4);
	return sqrt(n2) * 60;
}

float Core::Propeller::RPMToThrust(float rpm) {
	const float n2 = (rpm / 60) * (rpm / 60);
	return Ct * rho * n2 * D4;
}

float Core::Propeller::TorqueToRPM(float torque) {
	const float n2 = torque * 2 * 3.141f / (rho * Cp * D5);
	return sqrt(n2) * 60;
}

void Core::Propeller::ClearData(void) {
	d->nRPMs = 0;
	Log::Uart(Log::Lvl::Inf, "Cleared prop data");
}

static bool AddEntry(Core::Propeller::RPMTable *t, Core::Propeller::CoefficientEntry &e) {
	// find correct position
	uint8_t i;
	for (i = 0; i < t->nentries; i++) {
		if (t->entries[i].J >= e.J) {
			// found the position
			break;
		}
	}
	if (t->nentries > 0 && i < t->nentries && t->entries[i].J == e.J) {
		// perfect match, nothing to do for now
	} else {
		// need room for next entry
		if (t->nentries < Core::Propeller::MaxDiffAdvance) {
			// move the following entries
			const uint8_t nEntriesToMove = t->nentries - i;
			memmove(&t->entries[i + 1], &t->entries[i],
					nEntriesToMove * sizeof(e));
			t->nentries++;
		} else {
			// already at maximum capacity
			Log::Uart(Log::Lvl::Err,
					"Unable to add prop coefficient: max Js reached");
			return false;
		}
	}
	// write the entry
	t->entries[i] = e;
	return true;
}

bool Core::Propeller::ParseDataLine(uint16_t rpm, char* line) {
	float extract[4];
	for(uint8_t i=0;i<4;i++) {
		extract[i] = strtof(line, &line);
		if(!line) {
			Log::Uart(Log::Lvl::Err, "Failed to parse prop coeff line: %s", line);
			return false;
		}
	}

	CoefficientEntry e;
	e.J = extract[0];
	e.Ct = extract[1];
	e.Cp = extract[2];
	e.Eta = extract[3];

	// find appropriate RPM table
	uint8_t i;
	for (i = 0; i < d->nRPMs; i++) {
		if (d->rpm[i].rpm >= rpm) {
			// found correct position
			break;
		}
	}
	if (d->nRPMs > 0 && i < d->nRPMs && d->rpm[i].rpm == rpm) {
		// perfect match, nothing to do for now
	} else {
		// need room for next entry
		if (d->nRPMs < Core::Propeller::MaxDiffRPMs) {
			// move the following tables
			const uint8_t nTablesToMove = d->nRPMs - i;
			memmove(&d->rpm[i + 1], &d->rpm[i],
					nTablesToMove * sizeof(RPMTable));
			// initialize the new table
			d->rpm[i].nentries = 0;
			d->rpm[i].rpm = rpm;
			d->nRPMs++;
		} else {
			// already at maximum capacity
			Log::Uart(Log::Lvl::Err, "Unable to add prop coefficient: max RPMs reached");
			return false;
		}
	}
	return AddEntry(&d->rpm[i], e);
}

void Core::Propeller::PrintData(void) {
	for(uint8_t i=0;i<d->nRPMs;i++) {
		Log::Uart(Log::Lvl::Inf, "RPM: %d", d->rpm[i].rpm);
		for(uint8_t j=0;j<d->rpm[i].nentries;j++) {
			Log::Uart(Log::Lvl::Inf, "%6.4f %6.4f %6.4f %6.4f",
					d->rpm[i].entries[j].J, d->rpm[i].entries[j].Ct,
					d->rpm[i].entries[j].Cp, d->rpm[i].entries[j].Eta);
			vTaskDelay(10);
		}
	}
}

float Core::Propeller::RPMToTorque(float rpm) {
	const float n2 = (rpm / 60) * (rpm / 60);
	return Cp * rho * n2 * D5 / (2 * 3.141f);
}
