/*
    Author: Jayden Chan
    Date Created: Mar 4 2018
    Last Modified: Mar 4 2018
    Details: LED controller for the robot
*/

#include "LightHouse.h"

void performScan() {
    while(SensorValue[towerPot] < LIGHTHOUSE_UPPER) {
        if(SensorValue[lightSensor2] > highestValue) {
            highestValue = SensorValue[lightSensor2];
            pos = SensorValue[towerPot];
        }
        motor[towerMotor] = 20;
    }

    writeDebugStreamLine("val: %d", highestValue);
    writeDebugStreamLine("pos: %d", pos);

    while(SensorValue[towerPot] > pos) {
        motor[towerMotor] = -20;
    }

    motor[towerMotor] = 0;
    wait1Msec(500);

    while(SensorValue[towerPot] > LIGHTHOUSE_LOWER) {
        motor[towerMotor] = -20;
    }

    motor[towerMotor] = 0;
}
