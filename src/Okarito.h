#ifndef OKARITO_H
#define OKARITO_H

#include "DriveBase.c"
#include "RobotStates.h"

void testPeriodic();
void connect();
float averageLightSensor();
void drivePeriodic();

RobotState currentState = STATE_DISABLED;
float last[5];
float average;

#endif
