/* File: controller.cpp
 * Desc: Program that handles car movement and message handling.
 * Copyright (c) 2020 Adam Peterson - All rights reserved
*/

#include <Arduino.h>
#include <Servo.h>

#include "commands.h"
#include "controller.h"
#include "parameters.h"

bool connected = false;
byte motor = 0;
byte steer = STEERING_MID;
Servo servo;

void setup() {
  // Initialize serial, motor and serial control.
  Serial.begin(BAUD_RATE);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  servo.attach(STEERING_PIN);
  servo.write(STEERING_MID);
  // Wait until a connection between Arduino and computer is established.
  while(!connected) {
    write_command(HELLO);
    fill_buffer(1, 1000);
    get_message();
  }
}


void loop() {
  get_message();
  execute_command();
}


void execute_command() {
  servo.write(constrain(steer, STEERING_MIN, STEERING_MAX));
  motor = constrain(motor, MOTOR_MIN, MOTOR_MAX);
  if (motor < 0) {
    digitalWrite(DIRECTION_PIN, LOW);
  }
  else {
    digitalWrite(DIRECTION_PIN, HIGH);
  }
  analogWrite(MOTOR_PIN, motor);
}


Command read_command() {
  return (Command) Serial.read();
}


void fill_buffer(int n, unsigned long timeout) {
  unsigned long startTime = millis();
  while ((Serial.available() < n) && (millis() - startTime < timeout)) {}
}


byte read_byte() {
  fill_buffer(1, 100);
  return (byte) Serial.read();
}


void write_byte(byte n) {
  Serial.write(n);
}


void write_command(Command c) {
  Serial.write(c);
}


void debug(Command c) {
  write_command(c);
  switch(c) {
    case STOP:
      //fall through
    case MOTOR:
      write_byte(motor);
      break;
    case STEER:
      write_byte(steer);
      break;
  }
}


void get_message() {
  if (Serial.available() > 0) {
    Command command = read_command();
    if (!connected && command == HELLO) {
      connected = true;
    }
    else {
      switch(command) {
        case MOTOR:
          motor = read_byte;
          break;
        case STEER:
          steer = read_byte;
          break;
        case STOP:
          motor = 0;
          analogWrite(MOTOR_PIN, motor); // Milliseconds count!
          break;
        default:
          write_command(ERROR);
      } // End of switch
      write_command(OVER);
      if (DEBUG) {
        debug(command);
      }
    } // End of connected
  } // End of Serial.available
}
