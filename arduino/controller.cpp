/* File: controller.cpp
 * Desc: Program that handles car movement and message handling.
 * Copyright (c) 2020 Adam Peterson
 */

#include <Arduino.h>
#include <Servo.h>
#include <commands.h>
#include <controller.h>
#include <parameters.h>

// Initial values for starting the car.
bool connected = false;
byte motor = 0;
byte steer = STEERING_MID;
byte reverse = 0;
Servo servo;

void setup() {
  // Initialize serial, motor and serial control.
  Serial.begin(BAUD_RATE);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  servo.attach(STEERING_PIN);
  servo.write(STEERING_MID);
  // Wait until a connection between Arduino and computer is established.
  while (!connected) {
    write_command(HELLO);
    fill_buffer(1, 1000);
    get_command();
  }
}

void loop() {
  get_command();
  execute_command();
}

void execute_command() {
  steer = constrain(steer, STEERING_MIN, STEERING_MAX);
  motor = constrain(motor, MOTOR_MIN, MOTOR_MAX);
  if (reverse) {
    digitalWrite(DIRECTION_PIN, HIGH);
  } else {
    digitalWrite(DIRECTION_PIN, LOW);
  }
  servo.write(steer);
  analogWrite(MOTOR_PIN, motor);
}

Command read_command() { return (Command)Serial.read(); }

void fill_buffer(int n, unsigned long timeout) {
  unsigned long startTime = millis();
  while ((Serial.available() < n) && (millis() - startTime < timeout)) {
  }
}

byte read_byte() {
  fill_buffer(1, 100);
  return (byte)Serial.read();
}

void write_byte(byte n) { Serial.write(n); }

void write_command(Command c) { Serial.write(c); }

void get_command() {
  if (Serial.available() > 0) {
    Command command = read_command();
    if (!connected) {
      if (command == HELLO) {
        connected = true;
      }
    } else {
      switch (command) {
        case HELLO:
          // Fall through
        case OVER:
          break;
        case MOTOR:
          motor = read_byte();
          if (DEBUG) {
            write_command(command);
            write_byte(motor);
          }
          break;
        case STEER:
          steer = read_byte();
          if (DEBUG) {
            write_command(command);
            write_byte(steer);
          }
          break;
        case REVERSE:
          reverse = read_byte();
          if (DEBUG) {
            write_command(command);
            write_byte(motor);
          }
          break;
        default:
          write_command(ERROR);
      }  // End of switch
    }    // End of connected
    write_command(OVER);
  }   // End of Serial.available
}
