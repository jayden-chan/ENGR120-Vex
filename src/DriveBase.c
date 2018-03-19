/**
 * Class for controlling the robot's drivetrain.
 * This file combines code from several other files
 * in order to tell the drivetrain what to do in
 * specific circumstances. All driving functions
 * are controlled with closed-loop PID controllers.
 * Most of them are also using L-R compensation PID
 * to ensure that the robot stays on its intended course.
 *
 * @author Jayden Chan
 * @date January 13, 2018
 */

#include "PIDController.c"
#include "Ultrasonic.c"
#include "Arm.c"
#include "LEDController.c"
#include "Constants.h"
#include "Lighthouse.c"

PID slavePID;
PID slave2PID;
PID ultrasonicPID;
PID turnPID;

/**
 * Initialization code for all of the PID
 * controllers associated with the drivebase.
 */
void driveInit() {

    PIDInit(slavePID, SLAVE_kP, SLAVE_kI, SLAVE_kD, 100, 0, SLAVE_kS, true, SLAVE_kR);
    PIDReset(slavePID);

    PIDInit(slave2PID, SLAVE_2_kP, SLAVE_2_kI, SLAVE_2_kD, 127, 0, SLAVE_2_kS, true, SLAVE_2_kR);
    PIDReset(slave2PID);

    PIDInit(ultrasonicPID, ULTRASONIC_kP, ULTRASONIC_kI, ULTRASONIC_kD, 127, 0, ULTRASONIC_kS, true, ULTRASONIC_kR);
    PIDReset(ultrasonicPID);

    PIDInit(turnPID, TURN_kP, TURN_kI, TURN_kD, 1227, 0, TURN_kS, true, TURN_kR);
    PIDReset(turnPID);

    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);
}

/**
 * Resets all encoders and PID loops.
 */
void driveReset() {
    resetMotorEncoder(rightMotor);
    resetMotorEncoder(leftMotor);

    PIDReset(slavePID);
    PIDReset(slave2PID);
    PIDReset(ultrasonicPID);
    PIDReset(turnPID);
}

/**
 * Sets the values for the left and right side
 * of the drivetrain. Written to simplify
 * drive functions.
 *
 * @param left  Power level for the left side.
 * @param right Power level for the right side.
 */
void setRaw(float left, float right) {
    motor[leftMotor]  = left;
    motor[rightMotor] = right;
}

/**
 * Stops the motors.
 */
void stopMotors() {
    motor[leftMotor]  = 0;
    motor[rightMotor] = 0;
}

/**
 * Drives in a perfectly straight line using
 * distance PID and L-R compensation PID.
 *
 * @param distance The distance in cm.
 * @param maxSpeed The max allowed speed.
 * @param safeRange The acceptable range around
 * the target to finish in.
 * @param safeThreshold The time required to be
 * inside the safe zone before exiting the
 * function.
 */
void driveStraight(int distance, int maxSpeed, int safeRange, int safeThreshold) {

    driveReset();

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nPgmTime - time;
        time = nPgmTime;

        float driveError = (distance * TICKS_PER_CM2) - getMotorEncoder(rightMotor);
        float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(slave2PID, driveError);
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

/**
 * Performs an arc turn with the specified
 * radius and max speed. Radius is measured
 * from the INSIDE wheel.
 *
 * @param radius The radius for the arc.
 * @param orientation The ending orientation
 * for the arc.
 * @param turnRight Whether to turn right.
 * @param safeRange The acceptable range to
 * end in.
 * @param safeThreshold The time required to be
 * inside the safe zone before exiting the
 * function.
 */
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
        dTime = nPgmTime - time;
        time = nPgmTime;

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

        float driveOut = PIDCalculate(slave2PID, outsideError);
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

/**
 * Approaches the target using the ultrasonic
 * sensor and terminates when the cable has
 * been connected successfully.
 */
void cableApproach() {

    driveReset();

    float photosensorDefaultValue = SensorValue[lightSensor];
    while(!(isCableDetached(photosensorDefaultValue))) {

        float driveError = getUltraSonic() - ULTRASONIC_THRESH;
        float slaveError = (getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor));

        float driveOut = PIDCalculate(ultrasonicPID, driveError);
        float slaveOut = PIDCalculate(slave2PID, slaveError);

        driveOut = clamp(driveOut, 40);
        slaveOut = clamp(slaveOut, 40);

        setRaw((driveOut + slaveOut), (driveOut - slaveOut));
    }

    toggleRainbow();
    stopMotors();
}

/**
 * Drives until the ultrasonic sensor has a
 * value of 40 cm. Used for getting close to
 * the beacon for a second scan.
 */
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

/**
 * Rotates in place for the specified number of
 * degrees.
 *
 * @param degrees The number of degrees to turn.
 * @param maxSpeed The max allowed speed during the turn.
 * @param safeRange The acceptable range to around the target to
 * finish the turn in.
 * @param safeThreshold The amount of time neede to be inside
 * the safe zone before exiting the function.
 */
void rotate(float degrees, float maxSpeed, int safeRange, int safeThreshold) {

    driveReset();
    float arcLength;

    arcLength = (MATH_PI * DRIVETRAIN_WIDTH) * (abs(degrees) / 360);

    int safeTime = 0;
    int time     = 0;
    int dTime    = 0;

    while(true) {

        dTime = nPgmTime - time;
        time = nPgmTime;

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

/**
 * Tracks the beacon in real time using the
 * lighthouse assembly as a slave PID controller
 * for the drivetrain and the ultrasonic sensor
 * as the primary error.
 *
 * @param maxSpeed The maximum speed allowed.
 * @return Whether or not the connection was successful.
 */
bool realTimeApproach(int maxSpeed) {

    driveReset();

    float outsideError, slaveError;
    float turnMagnitude = 1000;
    bool lastDir;
    float ratio;
    int failTime = 0;

    float photosensorDefaultValue = SensorValue[lightSensor];
    while(!(isCableDetached(photosensorDefaultValue))) {

        if(getSensorLeft() < 1500 && getSensorRight() < 1500) {
            failTime++;
            if(failTime > 75) {
                stopMotors();
                return false;
            }
        }
        else {
            failTime = 0;
        }

        autoTrackBeacon();

        float sensorAngle = SensorValue[towerPot] - POT_TRACKING_THRESH;
        if(abs(sensorAngle) > 10) {
            turnMagnitude = 700 /  sqrt(abs(sensorAngle);
        }
        else {
            turnMagnitude = -1;
        }

        bool turnRight = sign(sensorAngle) > 0;

        float insideSet = (2 * MATH_PI * turnMagnitude) * TICKS_PER_CM2;
        float outsideSet = (2 * MATH_PI * (turnMagnitude + DRIVETRAIN_WIDTH)) * TICKS_PER_CM2;

        if (turnMagnitude == -1) {
            ratio = 1;
        }
        else if(insideSet != 0) {
            ratio = outsideSet / insideSet;
        }


        outsideError = getUltraSonic() - ULTRASONIC_THRESH;

        if(turnRight) {
            //outsideError = outsideSet - getMotorEncoder(leftMotor);
            slaveError = getMotorEncoder(leftMotor) - getMotorEncoder(rightMotor) * ratio;
        }
        else {
            //outsideError = outsideSet - getMotorEncoder(rightMotor);
            slaveError = getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor) * ratio;
        }

        float driveOut = PIDCalculate(ultrasonicPID, outsideError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        writeDebugStreamLine("Drive out: %f", driveOut);

        if(turnRight) {
            setRaw((driveOut - slaveOut), ((driveOut / ratio) + slaveOut));
        }
        else {
            setRaw(((driveOut / ratio) + slaveOut), (driveOut - slaveOut));
        }
    }

    stopMotors();
    return true;
}

/**
 * Scans for the beacon by rotating the light/
 * sensor assembly 360 degrees and choosing
 * the highest recorded value from the light
 * sensor.
 */
void performScan() {
    while(getSensorLeft() < BEACON_FOUND_THRESH && getSensorRight() < BEACON_FOUND_THRESH) {
        setRaw(30, -30);
    }
    setRaw(-20, 20);
    wait1Msec(20);
    stopMotors();
}
