#ifndef CONSTANTS_H
#define CONSTANTS_H

// Type     Name                 Value                                       Units
const int   MAX_SPEED            = 127;                                      // n/a
const float TICKS_PER_DEG        = 7.08333333333;                            // ticks
const float TICKS_PER_ROT        = 627.2;                                    // ticks
const float WHEEL_CIRC           = 31.91858136;                              // cm
const float TICKS_PER_CM2        = (TICKS_PER_ROT / WHEEL_CIRC);             // ticks
const float DRIVETRAIN_WIDTH     = 21.78;                                    // cm
const float MATH_PI              = 3.14159265359;                            // n/a
const int   ULTRASONIC_THRESH    = 0;                                        // cm
const int   ULTRASONIC_THRESH_2  = 50;                                       // cm
const int   CABLE_SENSOR_DELTA   = 30;                                       // n/a
const int   LIGHTHOUSE_UPPER     = 2300;                                     // ticks
const int   LIGHTHOUSE_LOWER     = 0;                                        // ticks
const int   POT_TRACKING_THRESH  = 1350;                                     // ticks
const int   BEACON_FOUND_THRESH  = 3100;
const int   L_SENSOR_DIFF        = 0;                                        // n/a
      int   POT_OFFSET           = -570;                                     // ticks

// PID Constants

const float SLAVE_2_kP = 0.1;
const float SLAVE_2_kI = 0.1;
const float SLAVE_2_kD = 10;
const float SLAVE_2_kS = 99999;
const int   SLAVE_2_kR = 0;

const float SLAVE_kP = 0.05;
const float SLAVE_kI = 0.0;
const float SLAVE_kD = 10;
const float SLAVE_kS = 99999;
const int   SLAVE_kR = 1;

const float ULTRASONIC_kP = 1.5;
const float ULTRASONIC_kI = 0.02;
const float ULTRASONIC_kD = 175;
const float ULTRASONIC_kS = 0.2;
const int   ULTRASONIC_kR = 10;

const float LIGHTHOUSE_kP = 0.5;
const float LIGHTHOUSE_kI = 0.0;
const float LIGHTHOUSE_kD = 50;
const float LIGHTHOUSE_kS = 999;
const float LIGHTHOUSE_kR = 10;

const float TURN_kP = 4;
const float TURN_kI = 0;
const float TURN_kD = 20;
const float TURN_kS = 5;
const int   TURN_kR = 1;

#endif
