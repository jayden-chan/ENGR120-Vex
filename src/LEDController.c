/*
    Author: Jayden Chan
    Date Created: Mar 3 2018
    Details: LED controller for the robot
*/

/**
 * Toggles the red LED on or off
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
 * Toggles the rainbow LED on or off
 */
void toggleRainbow() {
    if(SensorValue[LED2]) {
        SensorValue[LED2] = 0;
    }
    else {
        SensorValue[LED2] = 1;
    }
}
