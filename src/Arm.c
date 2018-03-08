/**
 * Wrapper class for the arm mechanism.
 * This code is used to easily determine
 * information about the state of the
 * robot's arm.
 *
 * @author Jayden Chan, Cobey Hollier
 * @date February 16, 2018
 */

#include "Constants.h"

/**
 * Returns true or false depending on whether
 * or not the cable has been successfully
 * attached to the beacon. The function
 * does this by detecting a difference in the
 * sensor values. If the sensor value changes
 * by more than the specified value in the
 * constants file, we know the cable has been
 * connected successfully.
 *
 * @param defaultValue The sensor value when
 * the cable is still being held by the robot.
 * @return Whether the cable is connected or not.
 */
bool isCableDetached(float defaultValue) {
    float sensorDelta = abs(SensorValue[lightSensor] - defaultValue);
    return sensorDelta > CABLE_SENSOR_DELTA;
}
