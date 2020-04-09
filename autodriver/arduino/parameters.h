/* File: parameters.h
 * Desc: A collection of predefined constants regarding minimum and maximum motor values, serial
 *       baud rate and digital pins.
 * Copyright (c) 2020 Adam Peterson
*/

#ifndef PARAMETERS_H
#define PARAMETERS_H

// Debug parameter
#define DEBUG true

// Serial baud rate parameter
#define BAUD_RATE 9600

// Digital pin parameters
#define MOTOR_PIN 11
#define STEERING_PIN 9
#define DIRECTION_PIN 8

// Steering servo parameters
#define STEERING_MID 90
#define STEERING_MIN 65
#define STEERING_MAX 115
#define STEERING_TRIM 0

// PWM values for motor speed
#define MOTOR_MAX 255
#define MOTOR_MIN 0 //25 is real min

#endif /* PARAMETERS_H */
