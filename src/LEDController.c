/*
    Author: Jayden Chan
    Date Created: Mar 3 2018
    Details: LED controller for the robot
*/

//*********************************************
// Toggles the red LED on or off
//
// @PARAM none
// @RETURN none
//*********************************************
void toggleRed() {
    if(SensorValue[redLED]) {
        SensorValue[redLED] = 0;
    }
    else {
        SensorValue[redLED] = 1;
    }
}
