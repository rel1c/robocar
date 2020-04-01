/* File: commands.h
 * Desc: Enumerated type definition for the purposes of remote car control.
 * Copyright (c) 2020 Adam Peterson
*/

#ifndef COMMANDS_H
#define COMMANDS_H

enum Command {
  HELLO = 1,    // Sent to initialize communications between Arduino and computer.
  OVER = 2,     // Sent to signal that a message was received and executed.
  MOTOR = 3,    // Sent to signal a change in motor speed.
  STEER = 4,    // Sent to signal a change in steering direction.
  REVERSE = 5,  // Sent to signal a change in motor direction.
  ERROR = 6     // Sent to signal an error.
};

typedef enum Command Command;

#endif /* COMMANDS_H */
