# File: control.py
# Desc: Serial control for the car via Python.
# Copyright (c) 2020 Adam Peterson

from enum import Enum

# Constants
ENDIAN = 'little'

# Control commands
class Command(Enum):
  HELLO = 1    # Sent to initialize communications between Arduino and computer.
  OVER = 2     # Sent to signal that a message was received and executed.
  MOTOR = 3    # Sent to signal a change in motor speed.
  STEER = 4    # Sent to signal a change in steering direction.
  REVERSE = 5  # Sent to signal a change in motor direction.
  ERROR = 6    # Sent to signal an error.

def read_byte(s):
  b = s.read(1)
  return int.from_bytes(b, byteorder=ENDIAN, signed=False)

def read_command(s):
  return Command(get_byte(s))

def write_serial(s, msg):
  m = bytes(msg)
  s.write(m)

def parse_command(c):
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
    print('Error in parsing command:', c)
  return msg
