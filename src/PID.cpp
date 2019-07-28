#ifndef PID_CPP
#define PID_CPP

#include "main.h"
#include "PID.hpp"

PID
pidInit(/*PID pid,*/ float fKP, float fKI, float fKD, float fEpsilonInner, float fEpsilonOuter,float dInner, float dOuter) {
	PID pid;
//	pros::lcd::print(6, "PID initializing: %f\n", fKP);
	pid.kP = fKP;
//	pros::lcd::print(7, "PID initialized: %f\n", pid.kP);
	pid.kI = fKI;
	pid.kD = fKD;
	pid.EpsilonInner = fEpsilonInner;
	pid.EpsilonOuter = fEpsilonOuter;
	pid.sigma = 0;
	pid.lastValue = 0;
	pid.lastTime = pros::millis();
	pid.dInner = dInner;
	pid.dOuter = dOuter;

	return pid;
}

/**
* calculate pid output
*
* @param pid instance of PID structure
* @param fSetPoint set point of PID controller
* @param fProcessVariable sensor/feedback value
*
* @return output value constrained from -127 to 127
*/
float
pidCalculate(PID pid, float fSetPoint, float fProcessVariable) {
  	//pros::lcd::print(5, "kP: %f\n", pid.kP);
	if (fSetPoint != pid.lastTarget) {
		pid.sigma = 0;
		pid.lastTarget = fSetPoint;
	}
  float fDeltaTime = (float)(pros::millis() - pid.lastTime) / 1000.0;
	pid.lastTime = pros::millis();

	float fDeltaPV = 0;
	if(fDeltaTime > 0)
		fDeltaPV = (fProcessVariable - pid.lastValue) / fDeltaTime;
	pid.lastValue = fProcessVariable;

	float fError = fSetPoint - fProcessVariable;

	if(fabs(fError) > pid.EpsilonInner && fabs(fError) < pid.EpsilonOuter)
		pid.sigma += fError * fDeltaTime;

	if (fabs (fError) > pid.EpsilonOuter)
		pid.sigma = 0;

	float fOutput = fError * pid.kP
	+ pid.sigma * pid.kI
	- (((fabs(fSetPoint) - fabs(fError) > 0 && fabs(fError) < pid.dInner) || (fabs(fSetPoint) - fabs(fError) < 0 && fabs(fError) < pid.dOuter))? (fDeltaPV * pid.kD) : 0);

	fOutput = fabs(fOutput) > 127 ? 127 * fOutput/fabs(fOutput) : fOutput;
	return fOutput;
}

#endif
