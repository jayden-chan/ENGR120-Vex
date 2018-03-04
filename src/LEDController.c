/*
    Author: Jayden Chan
    Date Created: Mar 3 2018
    Last Modified: Mar 3 2018
    Details: LED controller for the robot
*/

void toggleRed() {
    if(SensorValue[redLED]) {
        SensorValue[redLED] = 0;
    }
    else {
        SensorValue[redLED] = 1;
    }
}
