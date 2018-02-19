/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Last Modified: Feb 19 2018
    Details: Main robot code for 'Okarito'
*/

#include "Okarito.h"

void testPeriodic() {
    arcTurn(30, 90, false, 20, 250);

    currentState = STATE_DISABLED;
}

void waitingForButtons() {
    if(SensorValue[topButton]) {
        currentState = STATE_DRIVE;
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
    rotate(90, 50, 10, 250);
    currentState = STATE_WAITING;
}

void approachTarget() {
    ultrasonicApproach();

    currentState = STATE_DEPART;
}

void departTarget() {
    driveStraight(-50, 38, 10, 250);
    arcTurn(25, -90, false, 20, 250);

    currentState = STATE_WAITING;
}

float averageLightSensor(){

    last[0] = last[1];
    last[1] = last[2];
    last[2] = last[3];
    last[3] = last[4];
    last[4] = SensorValue[lightSensor];

    average = (last[0] + last[1] + last[2] + last[3] + last[4]) / 5;
    return average;
}
