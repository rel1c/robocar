/* File: controller.h
 * Desc: Interface for car movement and message handling.
 * Copyright (c) 2020 Adam Peterson
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

void execute_command();

void stop();

Command read_command();

void fill_buffer();

byte read_byte();

void write_byte();

void write_command();

void get_command();

#endif /* CONTROLLER_H */
