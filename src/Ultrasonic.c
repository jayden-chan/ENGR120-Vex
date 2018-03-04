/*
    Author: Jayden Chan
    Date Created: Feb 16 2018
    Last Modified: Feb 16 2018
    Details: Wrapper class for the ultrasonic sensor.
*/

//*********************************************
// Returns the value of the ultrasonic sensor
// after doing some processing to avoid getting
// weird values such as negative distance or
// spikes to over 100.
//
// @PARAM none
// @RETURN The value of the ultrasonic sensor.
//*********************************************
float getUltraSonic() {
    return clamp2((float) SensorValue[ultrasonic], 0, 100);
}
