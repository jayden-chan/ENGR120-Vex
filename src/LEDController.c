/**
 * This class is for controlling the robot's
 * LEDs. It includes functions for easily
 * toggling the state of the robot's LEDs.
 *
 * @author Jayden Chan
 * @date March 3, 2018
 */

/**
 * Toggles the red LED on or off.
 */
void toggleRed() {
    if(SensorValue[LED1]) {
        SensorValue[LED1] = 0;
    }
    else {
        SensorValue[LED1] = 1;
    }
}

/**
 * Toggles the rainbow LED on or off.
 */
void toggleRainbow() {
    if(SensorValue[LED2]) {
        SensorValue[LED2] = 0;
    }
    else {
        SensorValue[LED2] = 1;
    }
}

/**
 * Turns off all LEDs on the robot.
 * Used in the initialization code for
 * the robot.
 */
void turnOffAll() {
    SensorValue[LED1] = 0;
    SensorValue[LED2] = 0;
}
