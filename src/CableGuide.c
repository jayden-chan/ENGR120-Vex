/**
 * This class provides code for controlling the
 * robot's cable guide mechanism which is
 * responsible for preventing the cable from
 * getting caught in the drivetrain as well as
 * to separate the cable from the robot at the
 * end of the connection sequence.
 *
 * @author Jayden Chan
 * @date March 22, 2018
 */

/**
 * Lowers the cable guide.
 */
void cableGuideDown() {
    motor[cableMotor] = 20;
    wait1Msec(340);
    motor[cableMotor] = 0;
}

/**
 * Raises the cable guide.
 */
void cableGuideUp() {
    motor[cableMotor] = -20;
    wait1Msec(340);
    motor[cableMotor] = 0;
}
