/*
    Author: Jayden Chan
    Date Created: Jan 13 2018
    Last Modified: Feb 22 2018
    Details: DriveBase controller for the robot
*/

#include "PIDController.c"
#include "Ultrasonic.c"
#include "Arm.c"

PID masterPID;
PID slavePID;
PID ultrasonicPID;

/****************************************************************/
/*                   Init and reset functions                   */
/****************************************************************/

void driveInit() {

    PIDInit(masterPID, MASTER_kP, MASTER_kI, MASTER_kD, 127, 0, MASTER_kS, true, MASTER_kR);
    PIDReset(masterPID);

    PIDInit(slavePID, SLAVE_kP, SLAVE_kI, SLAVE_kD, 100, 0, SLAVE_kS, true, SLAVE_kR);
    PIDReset(slavePID);

    PIDInit(ultrasonicPID, ULTRASONIC_kP, ULTRASONIC_kI, ULTRASONIC_kD, 127, 0, ULTRASONIC_kS, true, ULTRASONIC_kR);
    PIDReset(ultrasonicPID);

    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);
}

// Resets critical drivebase components to prepare for the next manoeuvre
void driveReset() {
    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);

    PIDReset(masterPID);
    PIDReset(slavePID);
    PIDReset(ultrasonicPID);
}

/****************************************************************/
/*                       Helper functions                       */
/****************************************************************/

// Simply sets the values for the left and right side of the drivetrain
void setRaw(float left, float right) {
    motor[leftMotor]  = left;
    motor[rightMotor] = right;
}

// Stops the motors
void stopMotors() {
    motor[leftMotor]  = 0;
    motor[rightMotor] = 0;
}

/****************************************************************/
/*                        Drive functions                       */
/****************************************************************/

// Drives perfectly straight for the distance given (in cm)
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

// Performs an arc turn with the radius and ending orientation provided
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

// Approaches the target and breaks when the cable has been connected
void ultrasonicApproach() {

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

    stopMotors();
}

// Rotates in place for the given number of degrees
void rotate(float degrees, float maxSpeed, int safeRange, int safeThreshold) {

    driveReset();

    float arcLength = (MATH_PI * DRIVETRAIN_WIDTH) * (degrees / 360);

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nSysTime - time;
        time = nSysTime;

        float driveError = (arcLength * TICKS_PER_CM2) - getMotorEncoder(rightMotor);
        float slaveError = abs(getMotorEncoder(rightMotor)) - abs(getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(masterPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        setRaw(-(driveOut + slaveOut), (driveOut - slaveOut));

        writeDebugStreamLine("driveError: %f", driveError);
        writeDebugStreamLine("slaveError: %f", slaveError);

        safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

        writeDebugStreamLine("safeTime: %d", safeTime);

        if(safeTime > safeThreshold) {
            break;
        }
    }

    stopMotors();
}
