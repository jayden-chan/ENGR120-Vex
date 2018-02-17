/*
    Author: Jayden Chan
    Date Created: Jan 12 2018
    Last Modified: Feb 16 2018
    Details: PID Controller
*/

#include "Utils.c"
#include "Constants.h"

typedef struct {
    float P, I, D;
    float error;
    float errorSum, lastError;
    int lastTime;
    bool zeroOnCross;
    float integralLimit, epsilon;
    float dTime;
} PID;

// "Constructor" for the PID controller
void PIDInit(PID &pid, float kP, float kI, float kD, float integralLimit, float epsilon, bool zeroOnCross) {

    pid.P = kP;
    pid.I = kI;
    pid.D = kD;

    pid.integralLimit = integralLimit;
    pid.epsilon = epsilon;
    pid.zeroOnCross = zeroOnCross;
}

// Resets the error values for the provided PID controller
void PIDReset(PID &pid) {

    pid.error = 0;
    pid.lastTime = 0;
    pid.dTime = 0;
    pid.errorSum = 0;
    pid.lastError = 0;
}

float PIDCalculate(PID &pid, float error) {

    // Update time
    pid.dTime = nSysTime - pid.lastTime;
    pid.lastTime = nSysTime;

    // Update error
    pid.lastError = pid.error;
    pid.error = error;

    // Calculate the change in error if the elapsed time is not zero
    float changeInError = pid.dTime != 0 ? (pid.error - pid.lastError) / pid.dTime : 0;

    // Reset the error sum when the error crosses zero if that setting is enabled
    if(pid.zeroOnCross && (sign(pid.error) != sign(pid.lastError))) {
        pid.errorSum = 0;
    }

    // Add the P and D values to the output sum
    float output = pid.P * pid.error + pid.D * changeInError;

    if(abs(output) < MAX_SPEED) {
        // Calculate the integral of error if it is above the epsilon threshold
        pid.errorSum = abs(pid.error) > pid.epsilon ? pid.errorSum + pid.error * pid.dTime : 0;
    }

    // Clamp the integral to the provided integral limit
    pid.errorSum = clamp(pid.errorSum, pid.integralLimit);

    // Add the I term to the sum
    output += pid.I * pid.errorSum;

    // Return the sum
    return output;
}
