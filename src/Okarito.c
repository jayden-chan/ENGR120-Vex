/**
 * This is the file which contains the
 * main robot code for the robot. All functions
 * from other files come together here to make
 * the robot actually perform the functions it
 * needs to.
 *
 * @author Jayden Chan, Cobey Hollier
 * @date February 16, 2018
 */

#include "DriveBase.c"
#include "RobotStates.h"
#include "LEDController.c"
#include "LightHouse.c"
#include "CableGuide.c"

RobotState currentState = STATE_ENABLED;

/**
 * Function used for testing only.
 */
void testPeriodic() {
    toggleRainbowLED();
    toggleRedLED();
    currentState = STATE_DISABLED;
}

/**
 * Callibrates the robot's lighthouse assembly
 * so that it reads the correct values every time.
 */
void callibrate() {
    if(SensorValue[button2]) {
        motor[towerMotor] = 20;
    }
    else if(SensorValue[limitSwitch]) {
        motor[towerMotor] = -20;
    }
    else {
        motor[towerMotor] = 0;
        currentState = STATE_WAITING;
    }
}

/**
 * Checks the two buttons on the robot to see
 * if they have been pressed. When a button is
 * pressed the function will change the state
 * of the robot to whatever we want.
 */
void waitingForButtons() {
    if(SensorValue[topButton]) {
        currentState = STATE_SCAN;
    }
    if(SensorValue[limitSwitch]) {
        currentState = STATE_RECALLIBRATE;
    }
    if(SensorValue[button2]) {
        currentState = STATE_RECALLIBRATE;
    }
}

/**
 * Performs the scan for the target object
 * then changes the state of the robot to
 * rotate towards the found object.
 */
void scanForBeacon() {
    scanPID(180, 100, 40, 200);
    currentState = STATE_ROTATE;
}

/**
 * Rotates the robot towards the beacon
 * using the sensor value obtained from
 * the scanForBeacon function.
 */
void rotateToBeacon() {
    rotate((180-posInDegs), 40, 20, 200);
    currentState = STATE_APPROACH;
}

/**
 * Approaches the beacon using an ultrasonic
 * sensor and terminates the approach when
 * the cable has been connected successfully.
 */
void approachTarget() {
    bool success = realTimeApproachNew(127);

    if(!success) {
        currentState = STATE_SCAN;
    }
    else {
        currentState = STATE_DEPART;
    }
}

/**
 * Backs away from the target and turns after
 * the cable has successfully been connected.
 */
void departTarget() {
    motor[towerMotor] = 0;
    quikBak();

    toggleRedLED();
    toggleRainbowLED();

    currentState = STATE_DISABLED;
}
