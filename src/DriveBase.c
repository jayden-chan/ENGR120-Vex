/*
	Author: Jayden Chan
	Date Created: Jan 13 2018
	Last Modified: Feb 6 2018
	Details: DriveBase controller for the robot
*/

#include "PIDController.c"

PID masterPID;
PID slavePID;

void driveInit()
{
	PIDInit(masterPID, 3, 0, 10, 127, 0, true);
	PIDReset(masterPID);

	PIDInit(slavePID, 1, 0.25, 6, 100, 0, true);
	PIDReset(slavePID);

	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);
}

void driveStraight(int setpoint, int maxSpeed, int safeRange, int safeThreshold)
{
	int safeTime = 0;
	int time = 0;
	int dTime = 0;

	while(true)
	{
		dTime = nSysTime - time;
		time = nSysTime;

		float driveError = (setpoint * TICKS_PER_CM2) - getMotorEncoder(rightMotor);
		float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

		float driveOut = PIDCalculate(masterPID, driveError);
		float slaveOut = PIDCalculate(slavePID, slaveError);

		driveOut = clamp(driveOut, maxSpeed);
		slaveOut = clamp(slaveOut, maxSpeed);

		motor[rightMotor] = (driveOut - slaveOut);
		motor[leftMotor] = (driveOut + slaveOut);

		writeDebugStreamLine("driveError: %f", driveError);
		writeDebugStreamLine("slaveError: %f", slaveError);

		safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

		writeDebugStreamLine("safeTime: %d", safeTime);

		if(safeTime > safeThreshold)
		{
			//break;
		}

	}

	motor[rightMotor] = 0;
	motor[leftMotor] = 0;
}

void arcTurn(float radius, float orientation, bool turnRight, int safeRange, int safeThreshold)
{
	float insideSet = (2 * MATH_PI * radius) * (orientation / 360) * TICKS_PER_CM2;
	float outsideSet = (2 * MATH_PI * (radius + DRIVETRAIN_WIDTH)) * (orientation / 360) * TICKS_PER_CM2;

	float ratio = outsideSet / insideSet;

	float outsideError, slaveError;

	int safeTime = 0;
	int time = 0;
	int dTime = 0;

	while(true)
	{
		dTime = nSysTime - time;
		time = nSysTime;

		if(turnRight)
		{
			outsideError = outsideSet - getMotorEncoder(leftMotor);
			slaveError = getMotorEncoder(leftMotor) - getMotorEncoder(rightMotor) * ratio;
		}
		else
		{
			outsideError = outsideSet - getMotorEncoder(rightMotor);
			slaveError = getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor) * ratio;

			writeDebugStreamLine("Outside Error: %d", outsideError);
			writeDebugStreamLine("Slave Error: %d", slaveError);
		}

		float driveOut = PIDCalculate(masterPID, outsideError);
		float slaveOut = PIDCalculate(slavePID, slaveError);

		driveOut = clamp(driveOut, 70);
		slaveOut = clamp(slaveOut, 127);

		if(turnRight)
		{
			motor[leftMotor] = driveOut - slaveOut;
			motor[rightMotor] = (driveOut / ratio) + slaveOut;
		}
		else
		{
			motor[leftMotor] = (driveOut / ratio) + slaveOut;
			motor[rightMotor] = driveOut - slaveOut;
		}

		safeTime = abs(outsideError) < safeRange ? safeTime + dTime : 0;

		if(safeTime > safeThreshold)
		{
			break;
		}
	}
}
