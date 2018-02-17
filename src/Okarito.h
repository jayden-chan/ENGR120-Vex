#ifndef OKARITO_H
#define OKARITO_H

#include "DriveBase.c"

void testPeriodic(); //d
void connect(); //d
float averageLightSensor();
void drivePeriodic(); //d

float last[5];
float average, lastAverage;

#endif
