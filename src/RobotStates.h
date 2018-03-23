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
    STATE_RECALLIBRATE,
    STATE_SCAN,
    STATE_ROTATE,
    STATE_APPROACH,
    STATE_DEPART,
    STATE_TEST
} RobotState;

#endif
