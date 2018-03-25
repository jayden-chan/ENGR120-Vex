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
#include "DriveBase.c"

PID lightPID;

int pos = 0;
int posInDegs = 0;
float averageOne[3];
float averageTwo[3];
int numAverages = 3;

float highestValue = 0;

/**
 * Gets the value of the left light sensor
 * after averaging the last few values.
 *
 * @return The value of the sensor.
 */
float getLeftLight() {
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

/**
 * Gets the value of the right light sensor
 * after averaging the last few values.
 *
 * @return The value of the sensor.
 */
float getRightLight() {
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

        float error = ((degrees * TICKS_PER_DEG)) - (SensorValue[towerPot] + POT_OFFSET);
        float out = PIDCalculate(lightPID, error);

        out = clamp(out, maxSpeed);
        motor[towerMotor] = out;

        safeTime = abs(error) < safeRange ? safeTime + dTime : 0;

        if(safeTime > safeThreshold) {
            break;
        }
    }

    motor[towerMotor] = 0;
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
 * Scans for the target object less methodically
 * by simply stopping when the light sensors
 * exceed a certain threshold. This is much faster
 * than scanning the entire field of view but can
 * sometimes result in some error.
 */
void fastScan() {
    while(getLeftLight() < BEACON_FOUND_THRESH && getRightLight() < BEACON_FOUND_THRESH) {
        motor[towerMotor] = 20;
    }
    motor[towerMotor] = 0;

    pos = SensorValue[towerPot];
    posInDegs = (float)(pos-745) / TICKS_PER_DEG;

    rotateToDeg(200, 30, 100, 100);
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
    float diff = getLeftLight() - (getRightLight() + L_SENSOR_DIFF);

    if(abs(diff) > 0) {
        motor[towerMotor] = sign(diff) * -17;
    }
    else {
        motor[towerMotor] = 0;
    }
}

/**
 * it's like autoTrackBeacon.... but better...
 */
void betterAutoTrack() {
    float left = getLeftLight();
    float right = getRightLight();
    float diff = left - (right + L_SENSOR_DIFF);

    if(left < BEACON_LOST_THRESH && right < BEACON_LOST_THRESH) {
        motor[towerMotor] = 0;
    }
    else if(abs(diff) < 0) {
        motor[towerMotor] = 0;
    }
    else {
        // Activation function to get the motor to track
        // the target object smoothly. Determined experimentally.
        motor[towerMotor] = (diff * -TRACKING_SLOPE) - (TRACKING_MIN * sign(diff));
    }
}

void fastCheck() {
    while(SensorValue[towerPot] < POT_TRACKING_THRESH) {
        float val = getLeftLight();
        if(val > highestValue) {
            highestValue = val;
            pos = SensorValue[towerPot];
        }
        motor[towerMotor] = 127;
    }
    motor[towerMotor] = -127;
    wait1Msec(37);
    motor[towerMotor] = 0;
    posInDegs = (float)(pos+POT_OFFSET) / TICKS_PER_DEG;
}
