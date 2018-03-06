/*
    Author: Jayden Chan
    Date Created: Mar 4 2018
    Details: LED controller for the robot
*/

#include "LightHouse.h"


//*********************************************
// Initialization code for the lighthouse
// assembly PID controller.
//
// @PARAM none
// @RETURN none
//*********************************************
void lightHouseInit() {
    PIDInit(lightPID, LIGHTHOUSE_kP, LIGHTHOUSE_kI, LIGHTHOUSE_kD, 127, 0, LIGHTHOUSE_kS, true, LIGHTHOUSE_kR);
    PIDReset(lightPID);
}

//*********************************************
// Scans for the beacon by rotating the light/
// sensor assembly 360 degrees and choosing
// the highest recorded value from the light
// sensor.
//
// @PARAM none
// @RETURN none
//*********************************************
void performScan() {
    highestValue = 0;
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

    posInDegs = (float)(pos+POT_OFFSET) / TICKS_PER_DEG;

    rotateToDeg((float)pos/TICKS_PER_DEG, 20, 60, 250);

    motor[towerMotor] = 0;
}

//*********************************************
// Scans for the beacon in the opposite
// direction. Used to get a more precise
// measurement after the first scan / drive
// sequence is finished.
//
// @PARAM none
// @RETURN none
//*********************************************
void performReverseScan() {
    highestValue = 0;
    while(SensorValue[towerPot] > LIGHTHOUSE_LOWER) {
        if(SensorValue[lightSensor2] > highestValue) {
            highestValue = SensorValue[lightSensor2];
            pos = SensorValue[towerPot];
        }
        motor[towerMotor] = -20;
    }

    //writeDebugStreamLine("val: %d", highestValue);
    //writeDebugStreamLine("pos: %d", pos);
    //writeDebugStreamLine("in degs: %f", (float)pos / TICKS_PER_DEG);

    posInDegs = (float)(pos+POT_OFFSET) / TICKS_PER_DEG;

    rotateToDeg((float)pos/TICKS_PER_DEG, 20, 60, 250);

    motor[towerMotor] = 0;
}

//*********************************************
// Rotates the lighthouse assembly to a
// specific angle relative to the back of the
// robot using a PID loop.
//
// @PARAM degrees       The angle to rotate to
// @PARAM maxSpeed      The max allowed speed
// @PARAM safeRange     The range tollerance
// @PARAM safeThreshold The time needed to be
// in the safe zone before finishing
// @RETURN none
//*********************************************
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
