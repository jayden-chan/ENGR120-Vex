#ifndef ROBOTSTATES_H
#define ROBOTSTATES_H

typedef enum RobotStateEnum {
    STATE_DISABLED = 0,
    STATE_ENABLED,
    STATE_TEST,
    STATE_DRIVE
} RobotState;

#endif
