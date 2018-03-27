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
bool recovering = false;
float lastDir = 1;
int timeout = 0;

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

    if(recovering) {
        motor[towerMotor] = 15 * lastDir;
        if(left > BEACON_FOUND_THRESH) {
            recovering = false;
        }
    }
    else {
        if(left < BEACON_LOST_THRESH && right < BEACON_LOST_THRESH) {
            lastDir = sign(diff);
            recovering = true;
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
}

/**
 * Same as betterAutoTrack but with a
 * safety timeout in case it chooses the
 * wrong direction to recover in.
 * Experimental.
 */
void betterAutoTrackSafe() {
    float left = getLeftLight();
    float right = getRightLight();
    float diff = left - (right + L_SENSOR_DIFF);

    if(recovering) {
        motor[towerMotor] = 15 * lastDir;
        if(left > BEACON_FOUND_THRESH) {
            recovering = false;
            timeout++;
            if(timeout > 150) {
                lastDir *= -1;
            }
        }
    }
    else {
        if(left < BEACON_LOST_THRESH && right < BEACON_LOST_THRESH) {
            lastDir = sign(diff);
            recovering = true;
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
}

/**
 * Scans for the beacon using a PID loop instead
 * of just setting the motors for a certain amount
 * of time. Needed to ensure that the beacon is
 * exactly centered at the end of the scan.
 */
void scanPID(float degrees, int maxSpeed, int safeRange, int safeThreshold) {
    PIDReset(lightPID);

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;
    int offset   = 0;

    if(SensorValue[towerPot] < POT_TRACKING_THRESH) {
        offset = POT_OFFSET_LEFT;
    }
    else {
        offset = POT_OFFSET;
    }

    while(true) {

        dTime = nPgmTime - time;
        time = nPgmTime;

        float error = ((degrees * TICKS_PER_DEG)) - (SensorValue[towerPot] + offset);
        float out = PIDCalculate(lightPID, error);

        out = clamp(out, maxSpeed);
        motor[towerMotor] = out;

        safeTime = abs(error) < safeRange ? safeTime + dTime : 0;

        float val = getLeftLight();
        if(val > highestValue) {
            highestValue = val;
            pos = SensorValue[towerPot];
        }

        if(safeTime > safeThreshold) {
            break;
        }
    }

    posInDegs = (float)(pos+offset) / TICKS_PER_DEG;
    motor[towerMotor] = 0;
}
