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
    autoTrackBeacon();
}

/**
 * Callibrates the robot's lighthouse assembly
 * so that it reads the correct values every time.
 */
void callibrate() {
    if(SensorValue[button2]) {
        motor[towerMotor] = 20;
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
    rotateToDeg(0, 30, 40, 250);
    fastScan();
    wait1Msec(200);
    rotate(180-posInDegs, 30, 40, 250);
    currentState = STATE_APPROACH;
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
    bool success = realTimeApproach(40);

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

    driveStraight(-22, 30, 40, 250);
    cableGuideDown();
    rotate(30, 40, 70, 250);
    driveStraight(-22, 30, 40, 250);
    cableGuideUp();

    currentState = STATE_DISABLED;
}
