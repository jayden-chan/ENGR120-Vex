#ifndef LIGHTHOUSE_H
#define LIGHTHOUSE_H

#include "Constants.h"

void rotateToDeg(float degrees, int maxSpeed, int safeRange, int safeThreshold);

PID lightPID;
int highestValue = 0;
int pos = 0;
int posInDegs = 0;

#endif
