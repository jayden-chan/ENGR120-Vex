/*
    Author: Jayden Chan
    Date Created: Mar 4 2018
    Last Modified: Mar 4 2018
    Details: LED controller for the robot
*/

#include "LightHouse.h"

PID lightPID;

void lightHouseInit() {
    PIDInit(lightPID, LIGHTHOUSE_kP, LIGHTHOUSE_kI, LIGHTHOUSE_kD, 127, 0, LIGHTHOUSE_kS, true, LIGHTHOUSE_kR);
    PIDReset(lightPID);
}

void performScan() {
    while(SensorValue[towerPot] < LIGHTHOUSE_UPPER) {
        if(SensorValue[lightSensor2] > highestValue) {
            highestValue = SensorValue[lightSensor2];
            pos = SensorValue[towerPot];
        }
        motor[towerMotor] = 20;
    }

    writeDebugStreamLine("val: %d", highestValue);
    writeDebugStreamLine("pos: %d", pos);
    writeDebugStreamLine("in degs: %f", (float)pos / TICKS_PER_DEG);

    rotateToDeg((float)pos/TICKS_PER_DEG, 20, 20, 250);

    motor[towerMotor] = 0;
}

void rotateToDeg(float degrees, int maxSpeed, int safeRange, int safeThreshold) {
    PIDReset(lightPID);

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nSysTime - time;
        time = nSysTime;

        float error = ((degrees * TICKS_PER_DEG)) - SensorValue[towerPot];

        float out = PIDCalculate(lightPID, error);

        writeDebugStreamLine("Error: %f", error);

        out = clamp(out, maxSpeed);

        motor[towerMotor] = out;

        safeTime = abs(error) < safeRange ? safeTime + dTime : 0;

        if(safeTime > safeThreshold) {
            break;
        }
    }

    motor[towerMotor] = 0;
}
