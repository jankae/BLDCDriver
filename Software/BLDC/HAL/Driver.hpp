#pragma once

#include "BLDCHAL.hpp"
#include "lowlevel.hpp"
#include <array>

namespace HAL {
namespace BLDC {
class Driver : public HALDriver {
public:
	enum class Direction : uint8_t {
		Forward,
		Reverse,
	};

	using Data = struct data {
		std::array<uint16_t, 6> ZeroCal;
	};

	Driver(Data *d);
	~Driver();


	void SetPWM(int16_t promille) override;

	MotorData GetData() override;

	void FreeRunning();
	void BreakStop();

	State GetState();


	void InitiateStart() override;
	void Stop() override;

	void Calibrate();

	bool IsRunning();
	bool GotValidPosition();
	TestResult Test() override;

	uint32_t WindingResistance();
	uint16_t GetRPMSmoothed();
	uint16_t GetRPMInstant();

	static void DMAComplete();
	static void DMAHalfComplete();

	void SetDirection(Direction d) {
		dir = d;
	}

//private:
	enum class InternalState : uint8_t {
		None,					// only used to indicate no state change in stateBuf
		Stopped,				// all phases are actively pulled low, blocking the motor
		Align,					// low power align prior to starting (in case inductance sensing failed)
		AlignAndGo,
		Starting,
		Powered_PreZero,		// motor is running under power, waiting for zero crossing
		Powered_PastZero,		// motor is running under power, crossing already happened TODO remove, not longer needed
		Idle,					// unpowered, either stopped or running from external force/momentum
		Idle_Braking,			// regenerative braking active but DC bus can't take the charge -> idling
		Testing,				// Controller performs selftest, motor is not moving
		MeasuringResistance,
		Calibrating,
	};

	static Driver *Inst;
	static constexpr uint32_t minPWM = 100;
	static constexpr uint32_t CommutationTimeoutms = 100;
	static constexpr uint8_t MotorPoles = 12;
	static constexpr uint32_t startTime = 2000;
	void NewPhaseVoltages(uint16_t *data);
	void SetStep(uint8_t step);
	void SetIdle();
	void IncRotorPos();
	void WhileStateEquals(InternalState s);

	volatile InternalState IntState, stateBuf;
	uint32_t cnt;
	uint32_t timeToZero;

	Data *mot;

	LowLevel::Phase nPhaseHigh, nPhaseIdle;

	volatile int8_t RotorPos;
	Direction dir;

	uint32_t commutationCnt;
	uint32_t PWMperiodCnt;
	uint32_t lastStoppedTime;
	std::array<uint32_t, 6> CommutationCycles;
	uint32_t result;
};
}
}

