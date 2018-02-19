#ifndef OKARITO_H
#define OKARITO_H

#include "DriveBase.c"
#include "RobotStates.h"

void testPeriodic();
float averageLightSensor();
void waitingForButtons();
void waitingForApproach();
void driveOneMeter();
void turn90Degs();
void approachTarget();
void departTarget();

RobotState currentState = STATE_DISABLED;
float last[5];
float average;

#endif
