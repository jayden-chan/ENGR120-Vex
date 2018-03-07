/*
    Author: Jayden Chan
    Date Created: Feb 16 2018
    Details: Wrapper class for the ultrasonic sensor.
*/

int lastValue = 0;

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
