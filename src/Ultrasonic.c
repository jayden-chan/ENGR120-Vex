/*
    Author: Jayden Chan
    Date Created: Feb 16 2018
    Last Modified: Feb 16 2018
    Details: Wrapper class for the ultrasonic sensor.
*/

float getUltraSonic() {
    return clamp2((float) SensorValue[ultrasonic], 0, 100);
}
