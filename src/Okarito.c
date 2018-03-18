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

RobotState currentState = STATE_ENABLED;
bool scanned = false;

/**
 * Function used for testing only.
 */
void testPeriodic() {
    autoTrackBeacon();
}

/**
 * Callibrates the robot's lighthouse assembly
 * so that it reads the correct values every time.
 */
void callibrate() {
    if(SensorValue[button2]) {
        motor[towerMotor] = -20;
    }
    else {
        motor[towerMotor] = 0;
        currentState = STATE_WAITING;
    }
}

/**
 * Performs the scan for the target object
 * then changes the state of the robot to
 * rotate towards the found object.
 */
void scanForBeacon() {
    performScan();
    currentState = STATE_APPROACH;
}

/**
 * Takes the rotation value from the lighthouse
 * scan and rotates the robot to face it.
 */
void rotateTowardsBeacon() {
    writeDebugStreamLine("Rotating: %f", 180-posInDegs);

    rotate(((180-posInDegs)*1.0), 30, 15, 250);

    if(scanned) {
        toggleRed();
        currentState = STATE_DISABLED;
        return;
    }
    else {
        scanned = true;
        currentState = STATE_DISABLED;
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
        currentState = STATE_APPROACH;
    }
    if(SensorValue[button2]) {
        currentState = STATE_RECALLIBRATE;
    }
}

/**
 * Approaches the beacon using an ultrasonic
 * sensor and terminates the approach when
 * the cable has been connected successfully.
 */
void approachTarget() {
    realTimeApproach(30);

    currentState = STATE_DISABLED;
}

/**
 * Backs away from the target and turns after
 * the cable has successfully been connected.
 */
void departTarget() {
    driveStraight(-50, 38, 10, 250);
    //arcTurn(25, -90, true, 20, 250);

    currentState = STATE_WAITING;
}
