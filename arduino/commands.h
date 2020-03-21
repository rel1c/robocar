/** @file commands.h
 *
 * @brief Enumerated type definition for the purposes of remote car control.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2020 Adam Peterson. All rights reserved.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

enum Command {
  DEBUG = 0,
  ERROR = 1,
  MOTOR = 2,
  STEER = 3,
  STOP = 4,
  TRIM = 5
};

// Defined type so code is less verbose
typedef enum Command Command;

#endif /* COMMANDS_H */

/*** end of file ***/
