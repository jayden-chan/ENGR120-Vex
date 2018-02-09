#ifndef Constants.h
#define Constants.h

const int MAX_SPEED = 127;
const float TICKS_PER_DEG = 1.74222222; // ticks
const float TICKS_PER_ROT = 627.2; // ticks
const float WHEEL_CIRC = 31.91858136; // cm
const float DEGREES_PER_MVOLT = 0.0659340659; // degrees
const float DET_SHAFT_GEAR_RATIO = 36.0 / 60.0; // n/a
const float FINAL_POT_RATIO = DEGREES_PER_MVOLT * DET_SHAFT_GEAR_RATIO;
const float TICKS_PER_CM2 = (TICKS_PER_ROT / WHEEL_CIRC); // ticks
const float DRIVETRAIN_WIDTH = 21.95; // cm
const float MATH_PI = 3.14159265359; // n/a

#endif
