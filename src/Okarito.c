/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Last Modified: Feb 19 2018
    Details: Main robot code for 'Okarito'
*/

#include "Okarito.h"

void testPeriodic() {
    wait10Msec(30);
    driveStraight(50, 70, 20, 250);

    currentState = STATE_WAITING;
}

void waitingForButtons() {
    if(SensorValue[topButton]) {
        currentState = STATE_TEST;
    }
    if(SensorValue[button2]) {
        currentState = STATE_TURN;
    }
}

void waitingForApproach() {
    if(SensorValue[topButton]) {
        currentState = STATE_APPROACH;
    }
}

void driveOneMeter() {
    driveStraight(100, 70, 10, 250);
    currentState = STATE_WAITING;
}

void turn90Degs() {
    rotate(90, 30, 10, 250);
    currentState = STATE_WAITING;
}

void approachTarget() {
    ultrasonicApproach();

    currentState = STATE_DEPART;
}

void departTarget() {
    driveStraight(-50, 38, 10, 250);
    arcTurn(25, -90, true, 20, 250);

    currentState = STATE_WAITING;
}
