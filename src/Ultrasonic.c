/**
 * This is a wrapper class for the ultrasonic
 * sensor. It is responsible for filtering the
 * bad values from the sensor and making it easy
 * for us to get a clean value.
 *
 * @author Jayden Chan
 * @date February 16, 2018
 */

#include "Utils.c"

float lastOutput;
float dT;

/**
 * Prevents the ultrasonic sensor from
 * returning negative values as well as
 * clamping it to a set maximum.
 *
 * @return The value of the ultrasonic sensor.
 */
float getUltraSonic() {
    if(SensorValue[ultrasonic] == -1) {
        return 20;
    }

    return clamp((float)SensorValue[ultrasonic], 150);
}

/**
 * Applies slew rate filtering to the
 * ultrasonic output to avoid it spiking
 * like crazy.
 *
 * @return The filtered ultrasonic value.
 */
float getUltraSonicFiltered() {
    float toReturn = getUltraSonic();

    if(dT != 0) {
        if(abs(toReturn - lastOutput) / dT > ULTRASONIC_SLEW) {
            toReturn = lastOutput + slewRate * dT * sign(toReturn - lastOutput);
        }
    }

    lastOutput = toReturn;
    return toReturn;
}
