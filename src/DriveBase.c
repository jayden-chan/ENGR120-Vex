/*
    Author: Jayden Chan
    Date Created: Jan 13 2018
    Last Modified: Feb 13 2018
    Details: DriveBase controller for the robot
*/

#include "PIDController.c"

PID masterPID;
PID slavePID;

void driveInit() {

    PIDInit(masterPID, 3, 0, 10, 127, 0, true);
    PIDReset(masterPID);

    PIDInit(slavePID, 1, 0.25, 6, 100, 0, true);
    PIDReset(slavePID);

    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);
}

void driveReset() {
    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);

    PIDReset(masterPID);
    PIDReset(slavePID);
}

// Helper functions

void setRaw(float left, float right) {
    motor[leftMotor] = left;
    motor[rightMotor] = right;
}

void stopMotors() {
    motor[leftMotor] = 0;
    motor[rightMotor] = 0;
}

void driveStraight(int distance, int maxSpeed, int safeRange, int safeThreshold) {

    int safeTime = 0;
    int time = 0;
    int dTime = 0;

    while(true) {

        dTime = nSysTime - time;
        time = nSysTime;

        float driveError = (distance * TICKS_PER_CM2) - getMotorEncoder(rightMotor);
        float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(masterPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        setRaw((driveOut + slaveOut), (driveOut - slaveOut));

        //motor[leftMotor] = (driveOut + slaveOut);
        //motor[rightMotor] = (driveOut - slaveOut);

        writeDebugStreamLine("driveError: %f", driveError);
        writeDebugStreamLine("slaveError: %f", slaveError);

        safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

        writeDebugStreamLine("safeTime: %d", safeTime);

        if(safeTime > safeThreshold) {
            break;
        }

    }
    stopMotors();
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

    while(true) {
        dTime = nSysTime - time;
        time = nSysTime;

        if(turnRight) {
            outsideError = outsideSet - getMotorEncoder(leftMotor);
            slaveError = getMotorEncoder(leftMotor) - getMotorEncoder(rightMotor) * ratio;
        }
        else {
            outsideError = outsideSet - getMotorEncoder(rightMotor);
            slaveError = getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor) * ratio;

            writeDebugStreamLine("Outside Error: %d", outsideError);
            writeDebugStreamLine("Slave Error: %d", slaveError);
        }

        float driveOut = PIDCalculate(masterPID, outsideError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, 70);
        slaveOut = clamp(slaveOut, 127);

        if(turnRight) {
            setRaw((driveOut - slaveOut), ((driveOut / ratio) + slaveOut));
            //motor[leftMotor] = driveOut - slaveOut;
            //motor[rightMotor] = (driveOut / ratio) + slaveOut;
        }
        else {
            setRaw(((driveOut / ratio) + slaveOut), (driveOut - slaveOut));
            //motor[leftMotor] = (driveOut / ratio) + slaveOut;
            //motor[rightMotor] = driveOut - slaveOut;
        }

        safeTime = abs(outsideError) < safeRange ? safeTime + dTime : 0;

        if(safeTime > safeThreshold) {
            break;
        }
    }
}
