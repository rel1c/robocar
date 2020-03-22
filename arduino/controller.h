/* File: controller.h
 * Desc: Interface for car movement and message handling.
 * Copyright (c) 2020 Adam Peterson - All rights reserved
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

void execute_command();

void stop();

Command read_command();

void fill_buffer();

void read_char();

void write_char();

void write_command();

void get_messages();

#endif /* CONTROLLER_H */

/*** end of file ***/
