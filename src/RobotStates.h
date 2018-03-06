//**************************************
// List of all possible states for the
// robot's Finite State Machine.
//**************************************

#ifndef ROBOTSTATES_H
#define ROBOTSTATES_H

typedef enum RobotStateEnum {
    STATE_DISABLED,
    STATE_ENABLED,
    STATE_WAITING,
    STATE_TURN,
    STATE_DRIVE,
    STATE_APPROACH,
    STATE_DEPART,
    STATE_TEST,
    STATE_SCAN,
    STATE_SCAN2,
    STATE_ROTATE,
    STATE_GETCLOSE,
    STATE_RECALLIBRATE
} RobotState;

#endif
