#ifndef CONSTANTS_H
#define CONSTANTS_H

// Type            Name                Value                           Units
static const int   MAX_SPEED           = 127;                          //
static const float TICKS_PER_DEG       = 7.08333333333;                // ticks
static const float TICKS_PER_ROT       = 627.2;                        // ticks
static const float WHEEL_CIRC          = 31.91858136;                  // cm
static const float TICKS_PER_CM2       = (TICKS_PER_ROT / WHEEL_CIRC); // ticks
static const float DRIVETRAIN_WIDTH    = 21.78;                        // cm
static const float MATH_PI             = 3.14159265359;                //
static const float ULTRASONIC_THRESH   = 0;                            // cm
static const int   CABLE_SENSOR_DELTA  = 200;                          //
static const int   POT_TRACKING_THRESH = 2100;                         // ticks
static const int   BEACON_FOUND_THRESH = 2200;                         //
static const int   BEACON_LOST_THRESH  = 1500;                         //
             int   L_SENSOR_DIFF       = 0;                            //
static const int   POT_OFFSET          = -895;                         // ticks
static const int   POT_OFFSET_LEFT     = -800;                         // ticks
static const float TRACKING_SLOPE      = 0.007;                         //
static const float TRACKING_MIN        = 13.5;                         //
static const float TRACKING_TURN_SENS  = 400; // 170                          //
static const float ULTRASONIC_SLEW     = 0.8;                          //

// PID Constants
static const float SLAVE_2_kP = 0.1;
static const float SLAVE_2_kI = 0.1;
static const float SLAVE_2_kD = 10;
static const float SLAVE_2_kS = 99999;
static const int   SLAVE_2_kR = 0;

static const float SLAVE_kP = 0.05;
static const float SLAVE_kI = 0.0;
static const float SLAVE_kD = 10;
static const float SLAVE_kS = 99999;
static const int   SLAVE_kR = 1;

static const float ULTRASONIC_kP = 1.7;
static const float ULTRASONIC_kI = 0.02;
static const float ULTRASONIC_kD = 8000;
static const float ULTRASONIC_kS = 0.22;
static const int   ULTRASONIC_kR = 10;

static const float LIGHTHOUSE_kP = 0.5;
static const float LIGHTHOUSE_kI = 0.0;
static const float LIGHTHOUSE_kD = 50;
static const float LIGHTHOUSE_kS = 999;
static const float LIGHTHOUSE_kR = 10;

static const float TURN_kP = 1;
static const float TURN_kI = 0;
static const float TURN_kD = 100;
static const float TURN_kS = 5;
static const int   TURN_kR = 1;

#endif
