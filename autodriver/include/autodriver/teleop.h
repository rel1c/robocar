/* Predefined constants for teleop control */

#ifndef TELEOP_H
#define TELEOP_H
#include "autodriver/ControlState.h"

// int representation of input commands
#define KEY_UP 0x41
#define KEY_DOWN 0x42
#define KEY_RIGHT 0x43
#define KEY_LEFT 0x44
#define KEY_QUIT 0x71

// number of speed levels
#define NUM_SPD_INTRVL 5
// speed level -> percentage of motor use
#define SET_MOTOR_PCT(x) (float)abs(x) / float(NUM_SPD_INTRVL)

// amount of direction change per key press
#define TURN_ANGLE 5
#define STEER_MIN 65
#define STEER_MAX 115

// SIGINT handler
void on_exit(int sig);
// default message state
void init_state(autodriver::ControlState &msg);

#endif