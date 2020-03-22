/** @file commands.h
 *
 * @brief Enumerated type definition for the purposes of remote car control.
 *
 * COPYRIGHT NOTICE: (c) 2020 Adam Peterson. All rights reserved.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

enum Command {
  DEBUG = 0,  // Debug flag; instructs the Arduino to relay back commands given to it.
  ERROR = 1,  // Error flag; sent when there is a problem with messages.
  MOTOR = 2,  // Sent to signal a change in motor speed, followed by a value (-255, 255).
  STEER = 3,  // Sent to signal a change in steering direction, followed by a value (0, 180).
  STOP = 4,   // Sent to signal a complete stop of the motor.
  TRIM = 5    // Sent to signal a delta in steering bounds in case car isn't tracking straight.
};

// Defined type so code is less verbose
typedef enum Command Command;

#endif /* COMMANDS_H */

/*** end of file ***/
