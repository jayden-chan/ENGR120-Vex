/*
    Author: Jayden Chan
    Date Created: Mar 4 2018
    Last Modified: Mar 4 2018
    Details: LED controller for the robot
*/

void performScan() {
    while(SensorValue[towerPot] > LIGHTHOUSE_UPPER) {
        motor[towerMotor] = 10;
    }

    motor[towerMotor] = 0;
}

