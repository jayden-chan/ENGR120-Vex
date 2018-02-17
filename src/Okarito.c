/*
    Author: Jayden Chan, Cobey Hollier
    Date Created: Feb 16 2018
    Last Modified: Feb 16 2018
    Details: Main robot code
*/

void testPeriodic() {
    driveInit();
    arcTurn(10, 360, true, 20, 250);
    driveInit();
    driveStraight(75, 70, 20, 250);

    currentState = STATE_DISABLED;
}

void drivePeriodic() {
    driveStraight(-15, 30, 20, 250);
    driveInit();
    arcTurn(10, -90, true, 20, 250);
    currentState = STATE_DISABLED;
}

void connect() {
    float lightAverage;

    while(SensorValue[ultrasonic] > 10) {
        motor[leftMotor] = 30;
        motor[rightMotor] = 30;
        lightAverage = averageLightSensor();
    }

    while(averageLightSensor() - lightAverage < 30){
            motor[leftMotor] = 18;
            motor[rightMotor] = 18;
    }

    currentState = STATE_DRIVE;
}

float averageLightSensor(){

    last[0] = last[1];
    last[1] = last[2];
    last[2] = last[3];
    last[3] = last[4];
    last[4] = SensorValue[lightSensor];

    average = (last[0] + last[1] + last[2] + last[3] + last[4]) / 5;
    return average;
}

