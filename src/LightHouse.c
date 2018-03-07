/**
 * This class is responsible for controlling
 * the robot's "Lighthouse" assembly, which
 * includes a flashlight and visible light
 * (540 nm) phototransistor. It includes
 * functions for finding the beacon and
 * rotating the assembly to face it.
 *
 * @author Jayden Chan
 * @date March 4, 2018
 */

#include "LightHouse.h"

/**
 * Initialization code for the lighthouse
 * assembly PID controller.
 */
void lightHouseInit() {
    PIDInit(lightPID, LIGHTHOUSE_kP, LIGHTHOUSE_kI, LIGHTHOUSE_kD, 127, 0, LIGHTHOUSE_kS, true, LIGHTHOUSE_kR);
    PIDReset(lightPID);
}

/**
 * Scans for the beacon by rotating the light/
 * sensor assembly 360 degrees and choosing
 * the highest recorded value from the light
 * sensor.
 */
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

    //rotateToDeg((float)pos/TICKS_PER_DEG, 20, 60, 250);

    motor[towerMotor] = 0;
}

/**
 * Scans for the beacon in the opposite
 * direction. Used to get a more precise
 * measurement after the first scan / drive
 * sequence is finished.
 */
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

    //rotateToDeg((float)pos/TICKS_PER_DEG, 20, 60, 250);

    motor[towerMotor] = 0;
}

/**
 * Rotates the lighthouse assembly to a
 * specific angle relative to the back of the
 * robot using a PID loop.
 *
 * @param degrees       The angle to rotate to
 * @param maxSpeed      The max allowed speed
 * @param safeRange     The range tollerance
 * @param safeThreshold The time needed to be
 * in the safe zone before finishing
 */
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
