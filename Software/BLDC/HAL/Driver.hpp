#pragma once

#include "BLDCHAL.hpp"
#include "lowlevel.hpp"
#include <array>

namespace HAL {
namespace BLDC {
class Driver : public HALDriver {
public:
	Driver();
	~Driver();

	enum class State : uint8_t {
		None,					// only used to indicate no state change in stateBuf
		Stopped,				// all phases are actively pulled low, blocking the motor
		Align,					// low power align prior to starting (in case inductance sensing failed)
		Powered_PreZero,		// motor is running under power, waiting for zero crossing
		Powered_PastZero,		// motor is running under power, crossing already happened
		Idle,					// unpowered, either stopped or running from external force/momentum
		Idle_Braking,			// regenerative braking active but DC bus can't take the charge -> idling
		Testing,				// Controller performs selftest, motor is not moving
		MeasuringResistance,
	};

	enum class TestResult : uint8_t {
		OK,
		NoMotor,
		Failure,
	};

	enum class Direction : uint8_t {
		Forward,
		Reverse,
	};

	void SetPWM(int16_t promille) override;

	MotorData GetData() override;

	void FreeRunning();
	void Stop();

	State GetState();


	void InitiateStart() override;

	bool IsRunning();
	bool GotValidPosition();
	TestResult Test();

	uint32_t WindingResistance();
	uint16_t GetPWMSmoothed();
	uint16_t GetPWMInstant();

	static void DMAComplete();
	static void DMAHalfComplete();

	void SetDirection(Direction d) {
		dir = d;
	}

private:
	static Driver *Inst;
	static constexpr uint32_t minPWM = 100;
	static constexpr uint32_t CommutationTimeoutms = 50;
	static constexpr uint8_t MotorPoles = 12;
	void NewPhaseVoltages(uint16_t *data);
	void SetStep(uint8_t step);
	void SetIdle();
	void IncRotorPos();
	void WhileStateEquals(State s);

	State state, stateBuf;
	uint32_t cnt;
	uint32_t timeToZero;
	bool DetectorArmed;

	LowLevel::Phase nPhaseHigh, nPhaseIdle;

	int8_t RotorPos;
	Direction dir;

	uint32_t commutationCnt;
	uint32_t PWMperiodCnt;
	std::array<uint32_t, 6> CommutationCycles;
	uint32_t result;
};
}
}
