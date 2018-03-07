/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Details: Main robot code for 'Okarito'
*/

#include "Okarito.h"

/**
 * Function used for testing only.
 */
void testPeriodic() {
    toggleRed();
    currentState = STATE_WAITING;
}

/**
 * Begins the scan process when the top button
 * has been pressed.
 */
void waitingForScan() {
    if(SensorValue[topButton]) {
        scanned = false;
        currentState = STATE_SCAN;
    }
    if(SensorValue[button2]) {
        currentState = STATE_RECALLIBRATE;
    }
}

/**
 * Callibrates the robot's lighthouse assembly
 * so that it reads the correct values every time.
 */
void callibrate() {
    POT_OFFSET = 1075 - SensorValue[towerPot];
    writeDebugStreamLine("offset: %d", POT_OFFSET);
    currentState = STATE_WAITING;
}

/**
 * Performs the scan for the target object
 * then changes the state of the robot to
 * rotate towards the found object.
 */
void scanForBeacon() {
    performScan();
    currentState = STATE_ROTATE;
}

/**
 * Scans for the beacon in the opposite
 * direction after finding the initial angle.
 */
void scanForBeaconAgain() {
    performReverseScan();
    currentState = STATE_ROTATE;
}

/**
 * After the robot has rotated toward the
 * beacon, drive a little closer in order to
 * get a more accurate scan.
 */
void getClose() {
    ultrasonicApproach();
    currentState = STATE_SCAN2;
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
        currentState = STATE_GETCLOSE;
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
        currentState = STATE_DRIVE;
    }
    if(SensorValue[button2]) {
        currentState = STATE_TURN;
    }
}

/**
 * Checks if the button has been pressed, and
 * when it is, changes the robot state to
 * approach mode.
 */
void waitingForApproach() {
    if(SensorValue[topButton]) {
        currentState = STATE_APPROACH;
    }
}

/**
 * Drives exactly one meter using distance PID
 * and L-R compensation PID algorithms.
 */
void driveOneMeter() {
    driveStraight(100, 70, 10, 250);
    currentState = STATE_WAITING;
}

/**
 * Rotates exactly 90 degrees in place using
 * distance PID and L-R compensation PID
 * algorithms.
 */
void turn90Degs() {
    rotate(-90, 30, 10, 250);
    currentState = STATE_WAITING;
}

/**
 * Approaches the beacon using an ultrasonic
 * sensor and terminates the approach when
 * the cable has been connected successfully.
 */
void approachTarget() {
    cableApproach();

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
