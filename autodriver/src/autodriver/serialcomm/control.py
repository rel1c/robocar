# File: control.py
# Desc: Serial control for the car via Python.
# Copyright (c) 2020 Adam Peterson

from enum import Enum


# Control commands
class Command(Enum):
    """
    Commands to control the car, represented as an enum of integers.
    """
    HELLO = 1  # Sent to initialize communications between Arduino and computer.
    OVER = 2  # Sent to signal that a message was received and executed.
    MOTOR = 3  # Sent to signal a change in motor speed.
    STEER = 4  # Sent to signal a change in steering direction.
    REVERSE = 5  # Sent to signal a change in motor direction.
    ERROR = 6  # Sent to signal an error.

    def __repr__(self):
        return self.name + ':' + str(self.value)

    def __str__(self):
        return self.name


def read_byte(s):
    """
    Read in one byte from serial, and return it as an int.

    @param s: Serial object to be read.
    @returns: Byte received from serial.
    """
    b = s.read(1)
    return ord(b)


def read_command(s):
    """
    Read in one byte from serial, and return it as a Command object.

    @param s: Serial object to be read.
    @returns: Command object received from serial.
    @throws: ValueError in case byte is not a Command value.
    """
    values = tuple(c.value for c in Command)
    b = read_byte(s)
    if b in values:
        return Command(b)
    else:
        raise ValueError('Byte value (%d) read is not a Command.' % b)
