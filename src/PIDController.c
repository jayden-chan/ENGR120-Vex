/**
 * This class contains all the code needed
 * for the PID controllers on the robot. It
 * includes a struct typedef to enable "pseudo
 * object oriented" code in RobotC.
 *
 * @author Jayden Chan
 * @date January 16, 2018
 */

#include "Utils.c"
#include "Constants.h"

typedef struct {
    float P, I, D;
    float error;
    float errorSum, lastError;
    float output, lastOutput;
    int lastTime;
    bool zeroOnCross;
    float integralLimit, epsilon;
    float dTime;
    float slewRate;
    int iterations;
    int refreshRate;
} PID;

/**
 * Initializes the PID controller with the
 * provided values. There are many.
 *
 * @param pid The PID controller to initialize.
 * @param kP  The P value for the controller.
 * @param kI  The I value for the controller.
 * @param kD  The D value for the controller.
 * @param integralLimit The integral limit.
 * @param epsilon The epsilon value.
 * @param slewRate The slew rate.
 * @param zeroOnCross Whether or not to reset
 * the integral term when the error changes
 * signs.
 * @param refreshRate The delay between loops.
 */
void PIDInit(PID &pid, float kP, float kI, float kD, float integralLimit, float epsilon, float slewRate, bool zeroOnCross, int refreshRate) {

    pid.P = kP;
    pid.I = kI;
    pid.D = kD;

    pid.integralLimit = integralLimit;
    pid.epsilon = epsilon;
    pid.zeroOnCross = zeroOnCross;
    pid.slewRate = slewRate;

    pid.iterations = 0;
    pid.refreshRate = refreshRate;
}

/**
 * Resets all critical values for the PID
 * controller provided.
 *
 * @param pid The controller to reset.
 */
void PIDReset(PID &pid) {
    pid.error      = 0;
    pid.lastTime   = 0;
    pid.dTime      = 10; // dt is set to 10 to avoid divide by 0 error
    pid.errorSum   = 0;
    pid.lastError  = 0;
    pid.output     = 0;
    pid.lastOutput = 0;
    pid.iterations = 0;
}

/**
 * Filters the PID output. This includes
 * limiting the rate of change of the output
 * (slew rate) and ensuring that the output
 * does not exceed 127, the max value.
 *
 * @param pid The controller to filter.
 * @return The filtered output.
 */
float PIDFilter(PID &pid) {
    float toReturn = pid.output;

    if(pid.dTime != 0) {
        if(abs(pid.output - pid.lastOutput) / pid.dTime > pid.slewRate) {
            toReturn = pid.lastOutput + pid.slewRate * pid.dTime * sign(pid.output - pid.lastOutput);
        }
        else {
            toReturn = pid.output;
        }
    }

    toReturn = clamp(toReturn, MAX_SPEED);

    pid.lastOutput = toReturn;
    return toReturn;
}

/**
 * Computes the overall PID output using the
 * provided error. This function is pretty
 * complicated so I have included some extra
 * comments to explain what's going on.
 *
 * @param pid The PID controller to use.
 * @param error The error to use for the
 * calculation.
 * @return The output value of the PID controller.
 */
float PIDCalculate(PID &pid, float error) {

    pid.dTime = nPgmTime - pid.lastTime;
    pid.lastTime = nPgmTime;

    pid.lastError = pid.error;
    pid.error = error;

    if(pid.refreshRate != 0) {
        wait1Msec(pid.refreshRate);
    }

    float changeInError = pid.dTime != 0 ? (pid.error - pid.lastError) / pid.dTime : 0;

    if(pid.zeroOnCross && (sign(pid.error) != sign(pid.lastError))) {
        pid.errorSum = 0;
    }

    pid.output = pid.P * pid.error + pid.D * changeInError;

    if(abs(pid.output) < MAX_SPEED) {
        pid.errorSum = abs(pid.error) > pid.epsilon ? pid.errorSum + pid.error * pid.dTime : 0;
    }

    pid.errorSum = clamp(pid.errorSum, pid.integralLimit);
    pid.output += pid.I * pid.errorSum;
    pid.iterations++;

    if(pid.iterations > 5) {
        return PIDFilter(pid);
    }
    else {
        return 0;
    }
}
