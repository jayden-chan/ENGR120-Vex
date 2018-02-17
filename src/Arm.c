/*
    Author: Jayden Chan
    Date Created: Feb 16 2018
    Last Modified: Feb 16 2018
    Details: Wrapper class for arm mechanism
*/

bool isCableDetached() {
    return SensorValue[lightSensor] > 70;
}
