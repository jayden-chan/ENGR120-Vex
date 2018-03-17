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

#include "Constants.h"
#include "PIDController.c"
#include "Utils.c"

PID lightPID;
int highestValue = 0;
int pos = 0;
int posInDegs = 0;
float averageOne[3];
float averageTwo[3];
int numAverages = 3;

void rotateToDeg(float degrees, int maxSpeed, int safeRange, int safeThreshold);

float getSensorLeft() {
    for(int i = numAverages-1; i > 0; i--) {
        averageOne[i] = averageOne[i-1];
    }
    averageOne[0] = SensorValue[lightSensor2];

    float sum = 0;
    for(int i = 0; i < numAverages; i++) {
        sum += averageOne[i];
    }

    return sum / numAverages;
}

float getSensorRight() {
    for(int i = numAverages-1; i > 0; i--) {
        averageTwo[i] = averageTwo[i-1];
    }
    averageTwo[0] = SensorValue[rightLightSensor];

    float sum = 0;
    for(int i = 0; i < numAverages; i++) {
        sum += averageTwo[i];
    }

    return sum / numAverages;
}

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
 * Scans for the target object less methodically
 * by simply stopping when the light sensors
 * exceed a certain threshold. This is much faster
 * than scanning the entire field of view but can
 * sometimes result in some error.
 */
void fastScan() {
    while(getSensorLeft() < BEACON_FOUND_THRESH && getSensorRight() < BEACON_FOUND_THRESH) {
        motor[towerMotor] = 40;
    }
    motor[towerMotor] = -20;

    wait1Msec(40);
    motor[towerMotor] = 0;

    pos = SensorValue[towerPot];
    writeDebugStreamLine("offset: %d", POT_OFFSET);
    posInDegs = (float)(pos+POT_OFFSET) / TICKS_PER_DEG;

    rotateToDeg(180, 30, 80, 400);
}

/**
 * Automatically aligns the lighthouse assembly
 * with the beacon. The rotation value of the
 * lighthouse can then be used to automatically
 * adjust the heading of the robot to make sure
 * it stays on track at all times. This function
 * only works if the sensor is already mostly
 * aligned with the beacon; it's used mainly
 * for fine adjustment, not finding the beacon
 * from a dead start.
 */
void autoTrackBeacon() {
    float diff = getSensorLeft() - (getSensorRight() + L_SENSOR_DIFF);

    writeDebugStreamLine("Angle: %f", SensorValue[towerPot] - POT_TRACKING_THRESH);

    if(abs(diff) > 100) {
        motor[towerMotor] = sign(diff) * -20;
    }
    else {
        motor[towerMotor] = 0;
    }
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
 * @param degrees The angle to rotate to.
 * @param maxSpeed The max allowed speed.
 * @param safeRange The range tollerance.
 * @param safeThreshold The time needed to be
 * in the safe zone before finishing.
 */
void rotateToDeg(float degrees, int maxSpeed, int safeRange, int safeThreshold) {
    PIDReset(lightPID);

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nPgmTime - time;
        time = nPgmTime;

        float error = ((degrees * TICKS_PER_DEG)) - SensorValue[towerPot];
        float out = PIDCalculate(lightPID, error);

        //writeDebugStreamLine("Error: %f", error);

        out = clamp(out, maxSpeed);

        motor[towerMotor] = out;

        safeTime = abs(error) < safeRange ? safeTime + dTime : 0;

        if(safeTime > safeThreshold) {
            break;
        }
    }

    motor[towerMotor] = 0;
}
