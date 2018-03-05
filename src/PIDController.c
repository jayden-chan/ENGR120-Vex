/*
    Author: Jayden Chan
    Date Created: Jan 12 2018
    Details: PID Controller
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

// "Constructor" for the PID controller
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

// Resets the error values for the provided PID controller
void PIDReset(PID &pid) {
    pid.error      = 0;
    pid.lastTime   = 0;
    pid.dTime      = 10;
    pid.errorSum   = 0;
    pid.lastError  = 0;
    pid.output     = 0;
    pid.lastOutput = 0;
    pid.iterations = 0;
}

float PIDFilter(PID &pid) {
    float toReturn = pid.output;

    if(pid.dTime != 0) {
        if(abs(pid.output - pid.lastOutput) / pid.dTime > pid.slewRate) {
            toReturn = pid.lastOutput + pid.slewRate * pid.dTime * sign(pid.output);
        }
        else {
            toReturn = pid.output;
        }
    }

    toReturn = clamp(toReturn, 127);

    pid.lastOutput = toReturn;
    return toReturn;
}

float PIDCalculate(PID &pid, float error) {

    // Update time
    pid.dTime = nPgmTime - pid.lastTime;
    pid.lastTime = nPgmTime;

    // Update error
    pid.lastError = pid.error;
    pid.error = error;

    wait1Msec(pid.refreshRate);

    // Update output
    //pid.lastOutput = pid.output;

    // Calculate the change in error if the elapsed time is not zero
    float changeInError = pid.dTime != 0 ? (pid.error - pid.lastError) / pid.dTime : 0;

    // Reset the error sum when the error crosses zero if that setting is enabled
    if(pid.zeroOnCross && (sign(pid.error) != sign(pid.lastError))) {
        pid.errorSum = 0;
    }

    // Add the P and D values to the output sum
    pid.output = pid.P * pid.error + pid.D * changeInError;

    if(abs(pid.output) < MAX_SPEED) {
        // Calculate the integral of error if it is above the epsilon threshold
        pid.errorSum = abs(pid.error) > pid.epsilon ? pid.errorSum + pid.error * pid.dTime : 0;
    }

    // Clamp the integral to the provided integral limit
    pid.errorSum = clamp(pid.errorSum, pid.integralLimit);

    // Add the I term to the sum
    pid.output += pid.I * pid.errorSum;

    pid.iterations++;

    // Return the sum
    if(pid.iterations > 4) {
        return PIDFilter(pid);
    }
    else {
        return 0;
    }
}
