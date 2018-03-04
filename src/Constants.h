#ifndef CONSTANTS_H
#define CONSTANTS_H

// Type     Name                 Value                                       Units
const int   MAX_SPEED            = 127;                                      // n/a
const float TICKS_PER_DEG        = 11.944444444;                             // ticks
const float TICKS_PER_ROT        = 627.2;                                    // ticks
const float WHEEL_CIRC           = 31.91858136;                              // cm
const float TICKS_PER_CM2        = (TICKS_PER_ROT / WHEEL_CIRC);             // ticks
const float DRIVETRAIN_WIDTH     = 21.78;                                    // cm
const float MATH_PI              = 3.14159265359;                            // n/a
const int   ULTRASONIC_THRESH    = 3;                                        // cm
const int   CABLE_SENSOR_DELTA   = 30;                                       // n/a
const int   LIGHTHOUSE_UPPER     = 3950;                                     // ticks
const int   LIGHTHOUSE_LOWER     = 75;                                      // ticks

// PID Constants

const float MASTER_kP = 0.15;
const float MASTER_kI = 0.0;
const float MASTER_kD = 100;
const float MASTER_kS = 0.2;
const float MASTER_kR = 10;

const float SLAVE_kP = 0.1;
const float SLAVE_kI = 0.1;
const float SLAVE_kD = 10;
const float SLAVE_kS = 99999;
const int   SLAVE_kR = 1;

const float ULTRASONIC_kP = 2.5;
const float ULTRASONIC_kI = 0.08;
const float ULTRASONIC_kD = 3;
const float ULTRASONIC_kS = 999;
const int   ULTRASONIC_kR = 1;

const float LIGHTHOUSE_kP = 0.09;
const float LIGHTHOUSE_kI = 0;
const float LIGHTHOUSE_kD = 0;
const float LIGHTHOUSE_kS = 999;
const float LIGHTHOUSE_kR = 10;


#endif
