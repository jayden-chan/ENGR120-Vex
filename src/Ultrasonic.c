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

/**
 * Returns the value of the ultrasonic sensor
 * after doing some processing to avoid getting
 * weird values such as negative distance or
 * spikes to over 100.
 *
 * @return The value of the ultrasonic sensor.
 */
float getUltraSonic() {
    if(SensorValue[ultrasonic] == -1) {
        return 200;
    }

    lastValue = SensorValue[ultrasonic];
    return clamp2((float) SensorValue[ultrasonic], 0, 100);
}
