/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Last Modified: Feb 19 2018
    Details: Main robot code for 'Okarito'
*/

#include "Okarito.h"

//*********************************************
// Function used for testing only.
//
// @PARAM none
// @RETURN none
//*********************************************
void testPeriodic() {
    wait10Msec(30);
    driveStraight(50, 70, 20, 250);

    currentState = STATE_WAITING;
}

//*********************************************
// Checks the two buttons on the robot to see
// if they have been pressed. When a button is
// pressed the function will change the state
// of the robot to whatever we want.
//
// @PARAM none
// @RETURN none
//*********************************************
void waitingForButtons() {
    if(SensorValue[topButton]) {
        currentState = STATE_TEST;
    }
    if(SensorValue[button2]) {
        currentState = STATE_TURN;
    }
}

//*********************************************
// Checks if the button has been pressed, and
// when it is, changes the robot state to
// approach mode.
//
// @PARAM none
// @RETURN none
//*********************************************
void waitingForApproach() {
    if(SensorValue[topButton]) {
        currentState = STATE_APPROACH;
    }
}

//*********************************************
// Drives exactly one meter using distance PID
// and L-R compensation PID algorithms.
//
// @PARAM none
// @RETURN none
//*********************************************
void driveOneMeter() {
    driveStraight(100, 70, 10, 250);
    currentState = STATE_WAITING;
}

//*********************************************
// Rotates exactly 90 degrees in place using
// distance PID and L-R compensation PID
// algorithms.
//
// @PARAM none
// @RETURN none
//*********************************************
void turn90Degs() {
    rotate(90, 30, 10, 250);
    currentState = STATE_WAITING;
}

//*********************************************
// Approaches the beacon using an ultrasonic
// sensor and terminates the approach when
// the cable has been connected successfully.
//
// @PARAM none
// @RETURN none
//*********************************************
void approachTarget() {
    ultrasonicApproach();

    currentState = STATE_DEPART;
}

//*********************************************
// Backs away from the target and turns after
// the cable has successfully been connected.
//
// @PARAM none
// @RETURN none
//*********************************************
void departTarget() {
    driveStraight(-50, 38, 10, 250);
    arcTurn(25, -90, true, 20, 250);

    currentState = STATE_WAITING;
}
