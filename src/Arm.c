/*
    Author: Jayden Chan
    Date Created: Feb 16 2018
    Last Modified: Feb 16 2018
    Details: Wrapper class for arm mechanism
*/

bool isCableDetached(float defaultValue) {
    float sensorDelta = abs(SensorValue[lightSensor] - defaultValue);
    return sensorDelta > CABLE_SENSOR_DELTA;
}
