/** @file parameters.h
 *
 * @brief A collection of predefined constants regarding minimum and maximum motor values, serial
 *        baud rate and digital pins.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2020 Adam Peterson. All rights reserved.
*/

#ifndef PARAMETERS_H
#define PARAMETERS_H

// Serial baud rate parameter
#define BAUD_RATE

// Digital pin parameters
#define MOTOR_PIN 10
#define STEERING_PIN 9
#define DIRECTION_PIN 8

// Steering servo parameters
#define STEERING_MIN 0
#define STEERING_MAX 180
#define STEERING_TRIM 0

// Maximum PWM value for motor speed
#define MOTOR_MAX 255

// If DEBUG is set to true, all sent messages will be relayed back
#define DEBUG 0

#endif /* PARAMETERS_H */

/*** enf of file ***/
