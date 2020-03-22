/* File: commands.h
 * Desc: Enumerated type definition for the purposes of remote car control.
 * Copyright (c) 2020 Adam Peterson - All rights reserved
*/

#ifndef COMMANDS_H
#define COMMANDS_H

enum byte : Command {
  ERROR = 0,  // Error flag; sent when there is a problem with messages.
  HELLO = 1,  // Sent to initialize communications between Arduino and computer.
  MOTOR = 2,  // Sent to signal a change in motor speed, followed by a value (-255, 255).
  OVER = 3,   // Sent to signal that a message was received and executed.
  STEER = 4,  // Sent to signal a change in steering direction, followed by a value (0, 180).
  STOP = 5,   // Sent to signal a complete stop of the motor.
};

// Defined type so code is less verbose
typedef enum Command Command;

#endif /* COMMANDS_H */
