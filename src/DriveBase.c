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

float photosensorDefaultValue = 0;

/**
 * Initialization code for all of the PID
 * controllers associated with the drivebase.
 */
void driveInit() {

    // Initialize all of the PID controllers.
    PIDInit(slavePID, SLAVE_kP, SLAVE_kI, SLAVE_kD, 100, 0, SLAVE_kS, true, SLAVE_kR);
    PIDReset(slavePID);

    PIDInit(slave2PID, SLAVE_2_kP, SLAVE_2_kI, SLAVE_2_kD, 127, 0, SLAVE_2_kS, true, SLAVE_2_kR);
    PIDReset(slave2PID);

    PIDInit(ultrasonicPID, ULTRASONIC_kP, ULTRASONIC_kI, ULTRASONIC_kD, 127, 0, ULTRASONIC_kS, true, ULTRASONIC_kR);
    PIDReset(ultrasonicPID);

    PIDInit(turnPID, TURN_kP, TURN_kI, TURN_kD, 1227, 0, TURN_kS, true, TURN_kR);
    PIDReset(turnPID);

    // Set the default value for the cable attachment sensor
    photosensorDefaultValue = SensorValue[lightSensor];

    // Reset encoders.
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
        safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

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

    toggleRainbowLED();
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
    float arcLength = (MATH_PI * DRIVETRAIN_WIDTH) * (abs(degrees) / 360);

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

        safeTime = abs(driveError) < safeRange ? safeTime + dTime : 0;

        if(safeTime > safeThreshold) {
            break;
        }
    }

    stopMotors();
}

bool realTimeApproachNew(int maxSpeed) {
    driveReset();

    bool turnRight;
    float slaveError;
    float ratio = 1;

    bool wasRight = false;

    while(!(isCableDetached(photosensorDefaultValue))) {
        float driveError = getUltraSonicFiltered() - ULTRASONIC_THRESH;

        if(driveError < 40) {
            L_SENSOR_DIFF = -300;
        }

        turnRight = SensorValue[towerPot] > POT_TRACKING_THRESH;

        if(turnRight && !wasRight) {
            resetMotorEncoder(rightMotor);
            resetMotorEncoder(leftMotor);
            PIDReset(slavePID);
            wasRight = true;
        }
        else if(!turnRight && wasRight) {
            resetMotorEncoder(rightMotor);
            resetMotorEncoder(leftMotor);
            PIDReset(slavePID);
            wasRight = false;
        }

        ratio = (TRACKING_TURN_SENS + (abs(SensorValue[towerPot] - POT_TRACKING_THRESH))) / TRACKING_TURN_SENS;
        betterAutoTrack();

        if(turnRight) {
            slaveError = getMotorEncoder(leftMotor) - getMotorEncoder(rightMotor) * ratio;
        }
        else {
            slaveError = getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor) * ratio;
        }

        // Calculate the motor outputs using the PID controllers.
        float driveOut = PIDCalculate(ultrasonicPID, driveError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        // Limit the output of the PID controllers to the
        // specified max speed.
        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        writeDebugStreamLine("Speed: %f", driveOut);

        // Apply the power to the motors.
        if(turnRight) {
            setRaw((driveOut - slaveOut), ((driveOut / ratio) + slaveOut));
        }
        else {
            setRaw(((driveOut / ratio) + slaveOut), (driveOut - slaveOut));
        }
    }
    return true;
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

    while(!(isCableDetached(photosensorDefaultValue))) {

        // If the light sensors have been away from the beacon for
        // too long, abort the approach sequence and return false.
        if(getLeftLight() < 1500 && getRightLight() < 1500) {
            failTime++;
            if(failTime > 65) {
                stopMotors();
                return false;
            }
        }
        else {
            failTime = 0;
        }

        // Rotate the lighthouse assembly to automatically face the
        // target object.
        autoTrackBeacon();

        // Compute the turning radius based on the rotational value
        // of the lighthouse assembly.
        float sensorAngle = SensorValue[towerPot] - POT_TRACKING_THRESH;
        if(abs(sensorAngle) > 10) {
            turnMagnitude = 1200 /  sqrt(abs(sensorAngle));
        }
        else {
            turnMagnitude = -1;
        }

        // Check to see if we are going to be turning right.
        bool turnRight = sign(sensorAngle) > 0;

        // Compute the relative arc lengths for each side of the
        // drivetrain.
        float insideSet = (2 * MATH_PI * turnMagnitude) * TICKS_PER_CM2;
        float outsideSet = (2 * MATH_PI * (turnMagnitude + DRIVETRAIN_WIDTH)) * TICKS_PER_CM2;

        // Compute the ration between the two sides of the drivetrain.
        if (turnMagnitude == -1) {
            ratio = 1;
        }
        else if(insideSet != 0) {
            ratio = outsideSet / insideSet;
        }

        outsideError = getUltraSonic() - ULTRASONIC_THRESH;

        // Calculate the slave error (difference between the motors)
        // Helps to keep the robot on track.
        if(turnRight) {
            slaveError = getMotorEncoder(leftMotor) - getMotorEncoder(rightMotor) * ratio;
        }
        else {
            slaveError = getMotorEncoder(rightMotor) - getMotorEncoder(leftMotor) * ratio;
        }

        // Calculate the motor outputs using the PID controllers.
        float driveOut = PIDCalculate(ultrasonicPID, outsideError);
        float slaveOut = PIDCalculate(slavePID, slaveError);

        // Limit the output of the PID controllers to the
        // specified max speed.
        driveOut = clamp(driveOut, maxSpeed);
        slaveOut = clamp(slaveOut, maxSpeed);

        // Apply the power to the motors.
        if(turnRight) {
            setRaw((driveOut - slaveOut), ((driveOut / ratio) + slaveOut));
        }
        else {
            setRaw(((driveOut / ratio) + slaveOut), (driveOut - slaveOut));
        }
    }

    // Stop motors and return true because we know
    // we reached the beacon successfully.
    stopMotors();
    return true;
}
