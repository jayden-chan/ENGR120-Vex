#ifndef ROBOTSTATES_H
#define ROBOTSTATES_H

typedef enum RobotStateEnum {
    STATE_DISABLED,
    STATE_ENABLED,
    STATE_WAITING,
    STATE_TURN,
    STATE_DRIVE
} RobotState;

#endif
