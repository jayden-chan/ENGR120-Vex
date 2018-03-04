/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Last Modified: Feb 22 2018
    Details: Wrapper class for arm mechanism
*/

//*********************************************
// Returns true or false depending on whether
// or not the cable has been successfully
// attached to the beacon. The function
// does this by detecting a difference in the
// sensor values. If the sensor value changes
// by more than the specified value in the
// constants file, we know the cable has been
// connected successfully.
//
// @PARAM defaultValue The sensor value when
// the cable is still being held by the robot
// @RETURN Whether the cable is connected or not
//*********************************************
bool isCableDetached(float defaultValue) {
    float sensorDelta = abs(SensorValue[lightSensor] - defaultValue);
    return sensorDelta > CABLE_SENSOR_DELTA;
}
