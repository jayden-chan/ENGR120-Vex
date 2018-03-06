/*
    Author: Jayden Chan
    Date Created: Jan 13 2018
    Details: DriveBase controller for the robot
*/

#include "PIDController.c"
#include "Ultrasonic.c"
#include "Arm.c"
#include "LEDController.c"

PID masterPID;
PID slavePID;
PID ultrasonicPID;
PID turnPID;

/****************************************************************/
/*                   Init and reset functions                   */
/****************************************************************/

//*********************************************
// Initialization code for all of the PID
// controllers associated with the drivebase.
//
// @PARAM none
// @RETURN none
//*********************************************
void driveInit() {

    PIDInit(masterPID, MASTER_kP, MASTER_kI, MASTER_kD, 127, 0, MASTER_kS, true, MASTER_kR);
    PIDReset(masterPID);

    PIDInit(slavePID, SLAVE_kP, SLAVE_kI, SLAVE_kD, 100, 0, SLAVE_kS, true, SLAVE_kR);
    PIDReset(slavePID);

    PIDInit(ultrasonicPID, ULTRASONIC_kP, ULTRASONIC_kI, ULTRASONIC_kD, 127, 0, ULTRASONIC_kS, true, ULTRASONIC_kR);
    PIDReset(ultrasonicPID);

    PIDInit(turnPID, TURN_kP, TURN_kI, TURN_kD, 1227, 0, TURN_kS, true, TURN_kR);
    PIDReset(turnPID);

    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);
}

//*********************************************
// Resets all encoders and PID loops.
//
// @PARAM none
// @RETURN none
//*********************************************
void driveReset() {
    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);

    PIDReset(masterPID);
    PIDReset(slavePID);
    PIDReset(ultrasonicPID);
    PIDReset(turnPID);
}

/****************************************************************/
/*                       Helper functions                       */
/****************************************************************/

//*********************************************
// Sets the values for the left and right side
// of the drivetrain. Written to simplify
// drive functions.
//
// @PARAM left  Power level for the left side.
// @PARAM right Power level for the right side.
// @RETURN none
//*********************************************
void setRaw(float left, float right) {
    motor[leftMotor]  = left;
    motor[rightMotor] = right;
}

//*********************************************
// Stops the motors.
//
// @PARAM none
// @RETURN none
//*********************************************
void stopMotors() {
    motor[leftMotor]  = 0;
    motor[rightMotor] = 0;
}

/****************************************************************/
/*                        Drive functions                       */
/****************************************************************/

//*********************************************
// Drives in a perfectly straight line using
// distance PID and L-R compensation PID.
//
// @PARAM distance The distance in cm.
// @PARAM maxSpeed The max allowed speed.
// @param safeRange The acceptable range around
// the target to finish in.
// @PARAM safeThreshold The time required to be
// inside the safe zone before exiting the
// function.
// @RETURN none
//*********************************************
void driveStraight(int distance, int maxSpeed, int safeRange, int safeThreshold) {

    driveReset();

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nSysTime - time;
        time = nSysTime;

        float driveError = (distance * TICKS_PER_CM2) - getMotorEncoder(rightMotor);
        float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(masterPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        setRaw((driveOut + slaveOut), (driveOut - slaveOut));

        //writeDebugStreamLine("driveError: %f", driveError);
        //writeDebugStreamLine("slaveError: %f", slaveError);

        safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

        //writeDebugStreamLine("safeTime: %d", safeTime);

        if(safeTime > safeThreshold) {
            break;
        }
    }

    stopMotors();
}

//*********************************************
// Performs an arc turn with the specified
// radius and max speed. Radius is measured
// from the INSIDE wheel.
//
// @PARAM radius The radius for the arc.
// @PARAM orientation The ending orientation
// for the arc.
// @PARAM turnRight Whether to turn right
// @PARAM safeRange The acceptable range to
// end in.
// @PARAM safeThreshold The time required to be
// inside the safe zone before exiting the
// function.
// @RETURN none
//*********************************************
void arcTurn(float radius, float orientation, bool turnRight, int safeRange, int safeThreshold) {

    driveReset();

    float insideSet = (2 * MATH_PI * radius) * (orientation / 360) * TICKS_PER_CM2;
    float outsideSet = (2 * MATH_PI * (radius + DRIVETRAIN_WIDTH)) * (orientation / 360) * TICKS_PER_CM2;

    float ratio = outsideSet / insideSet;

    float outsideError, slaveError;

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {
        dTime = nSysTime - time;
        time = nSysTime;

        if(turnRight) {
            outsideError = outsideSet - getMotorEncoder(leftMotor);
            slaveError = getMotorEncoder(leftMotor) - getMotorEncoder(rightMotor) * ratio;
        }
        else {
            outsideError = outsideSet - getMotorEncoder(rightMotor);
            slaveError = getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor) * ratio;

            writeDebugStreamLine("Outside Error: %d", outsideError);
            writeDebugStreamLine("Slave Error: %d", slaveError);
        }

        float driveOut = PIDCalculate(masterPID, outsideError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, 70);
        slaveOut = clamp(slaveOut, 127);

        if(turnRight) {
            setRaw((driveOut - slaveOut), ((driveOut / ratio) + slaveOut));
        }
        else {
            setRaw(((driveOut / ratio) + slaveOut), (driveOut - slaveOut));
        }

        safeTime = abs(outsideError) < safeRange ? safeTime + dTime : 0;

        writeDebugStreamLine("Safe time: %d", safeTime);
        writeDebugStreamLine("Safe thresh: %d", safeThreshold);

        if(safeTime > safeThreshold) {
            break;
        }
    }

    stopMotors();
}

//*********************************************
// Approaches the target using the ultrasonic
// sensor and terminates when the cable has
// been connected successfully.
//
// @PARAM none
// @RETURN none
//*********************************************
void cableApproach() {

    driveReset();

    float photosensorDefaultValue = SensorValue[lightSensor];
    while(!(isCableDetached(photosensorDefaultValue))) {

        float driveError = getUltraSonic() - ULTRASONIC_THRESH;
        float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(ultrasonicPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, 30);
        slaveOut = clamp(slaveOut, 30);

        setRaw((driveOut + slaveOut), (driveOut - slaveOut));
    }

    toggleRainbow();
    stopMotors();
}

//*********************************************
// Drives until the ultrasonic sensor has a
// value of 40 cm. Used for getting close to
// the beacon for a second scan.
//
// @PARAM none
// @RETURN none
//*********************************************
void ultrasonicApproach() {

    driveReset();

    while(true) {

        float driveError = getUltraSonic() - ULTRASONIC_THRESH_2;
        float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(ultrasonicPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, 30);
        slaveOut = clamp(slaveOut, 30);

        setRaw((driveOut + slaveOut), (driveOut - slaveOut));

        if(driveError < 5) break;
    }

    stopMotors();
}

//*********************************************
// Rotates in place for the specified number of
// degrees.
//
// @PARAM none
// @RETURN none
//*********************************************
void rotate(float degrees, float maxSpeed, int safeRange, int safeThreshold) {

    driveReset();
    float arcLength;

    arcLength = (MATH_PI * DRIVETRAIN_WIDTH) * (abs(degrees) / 360);

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nSysTime - time;
        time = nSysTime;

        float driveError;

        if(degrees > 0) {
            driveError = (arcLength * TICKS_PER_CM2) - getMotorEncoder(rightMotor);
        }
        else {
            driveError = (arcLength * TICKS_PER_CM2) + getMotorEncoder(rightMotor);
        }

        float slaveError = abs(getMotorEncoder(rightMotor)) - abs(getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(turnPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        if(degrees > 0) {
            setRaw(-(driveOut + slaveOut), (driveOut - slaveOut));
        }
        else {
            setRaw((driveOut + slaveOut), -(driveOut - slaveOut));
        }

        //writeDebugStreamLine("driveError: %f", driveError);
        //writeDebugStreamLine("slaveError: %f", slaveError);

        safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

        //writeDebugStreamLine("safeTime: %d", safeTime);

        if(safeTime > safeThreshold) {
            break;
        }
    }

    stopMotors();
}
