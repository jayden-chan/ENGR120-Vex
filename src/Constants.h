#ifndef CONSTANTS_H
#define CONSTANTS_H

// Type     Name                 Value                                       Units
const int   MAX_SPEED            = 127;                                      // n/a
const float TICKS_PER_DEG        = 1.74222222;                               // ticks
const float TICKS_PER_ROT        = 627.2;                                    // ticks
const float WHEEL_CIRC           = 31.91858136;                              // cm
const float DEGREES_PER_MVOLT    = 0.0659340659;                             // degrees
const float DET_SHAFT_GEAR_RATIO = 36.0 / 60.0;                              // n/a
const float FINAL_POT_RATIO      = DEGREES_PER_MVOLT * DET_SHAFT_GEAR_RATIO; // n/a
const float TICKS_PER_CM2        = (TICKS_PER_ROT / WHEEL_CIRC);             // ticks
const float DRIVETRAIN_WIDTH     = 21.78;                                    // cm
const float MATH_PI              = 3.14159265359;                            // n/a
const int   ULTRASONIC_THRESH    = 3;                                        // cm
const int   CABLE_SENSOR_DELTA   = 30;                                       // n/a

// PID Constants

const float MASTER_kP = 0.8;
const float MASTER_kI = 0;
const float MASTER_kD = 10;
const float MASTER_kS = 999;

const float SLAVE_kP = 0.9;
const float SLAVE_kI = 0.25;
const float SLAVE_kD = 10;
const float SLAVE_kS = 999;

const float ULTRASONIC_kP = 3;
const float ULTRASONIC_kI = 0.08;
const float ULTRASONIC_kD = 3;
const float ULTRASONIC_kS = 999;

#endif
