/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Last Modified: Feb 16 2018
    Details: Main robot code for 'Okarito'
*/

#include "Okarito.h"

void testPeriodic() {
    driveReset();
    arcTurn(10, 360, true, 20, 250);
    driveReset();
    driveStraight(75, 70, 20, 250);

    currentState = STATE_DISABLED;
}

void drivePeriodic() {
    driveReset();
    driveStraight(-15, 30, 20, 250);
    driveReset();
    arcTurn(10, -90, true, 20, 250);
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

void driveOneMeter() {
    driveReset();
    driveStraight(100, 70, 10, 250);
    currentState = STATE_WAITING;
}

void turn90Degs() {
    driveReset();
    rotate(90, 50, 10, 250);
    currentState = STATE_WAITING;
}

void connect() {
    float lightAverage;

    while(getUltraSonic() > 10) {
        motor[leftMotor] = 30;
        motor[rightMotor] = 30;
        lightAverage = averageLightSensor();
    }

    while(averageLightSensor() - lightAverage < 30){
            motor[leftMotor] = 18;
            motor[rightMotor] = 18;
    }

    currentState = STATE_DRIVE;
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
