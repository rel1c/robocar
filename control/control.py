# File: control.py
# Desc: Serial control for the car via Python.
# Copyright (c) 2020 Adam Peterson

from enum import Enum

# Constants
ENDIAN = 'little'


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
        return self.name


def read_byte(s):
    """
    Read in one byte from serial, and return it as an int.

    @param s: Serial object to be read.
    @returns: Byte received from serial.
    """
    b = s.read(1)
    return int.from_bytes(b, byteorder=ENDIAN, signed=False)


def read_command(s):
    """
    Read in one byte from serial, and return it as a Command object.
  
    @param s: Serial object to be read.
    @returns: Command object received from serial.
    """
    b = read_bytes(s)
    if b < 1 or b > 6:
      return Command.ERROR
    # else
    return Command(b)


def write_serial(s, msg):
    """
    Write a list of bytes to serial.
  
    @param s: Serial object to be written.
    @param msg: List of bytes to be written to serial.
    """
    m = bytes(msg)
    s.write(m)


def parse_command(c):
    """
    Parse a Command into a string.
  
    @param c: The Command object to be parsed.
    @returns: A string representation of a Command.
    """
    try:
        command = Command(c)
        if command == Command.HELLO:
            msg = 'HELLO'
        elif command == Command.OVER:
            msg = 'OVER'
        elif command == Command.MOTOR:
            msg = 'MOTOR'
        elif command == Command.STEER:
            msg = 'STEER'
        elif command == Command.REVERSE:
            msg = 'REVERSE'
        elif command == Command.ERROR:
            msg = 'ERROR'
        else:
            msg = ''
    except Exception as e:
        print('Error in parsing command:', e)
    return msg
