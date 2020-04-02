# File: serial-test.py
# Desc: Serial command parser for executing car commands via the terminal.
# Copyright (c) 2020 Adam Peterson - All rights reserved

import serial

# Constants
BAUD_RATE = 9600
ENDIAN = 'little'
SERIAL_PATH = '/dev/ttyACM0'

# Commands
command_to_code = {'ERROR':0, 'HELLO':1, 'MOTOR':2, 'OVER':3, 'STEER':4, 'STOP':5}
code_to_command = {0:'ERROR', 1:'HELLO', 2:'MOTOR', 3:'OVER', 4:'STEER', 5:'STOP'}

# Serial
port = serial.Serial(SERIAL_PATH, BAUD_RATE)
port.timeout = 1

def get_code():
  command = input('Enter a command: ')
  command = command.upper()
  while command not in command_to_code:
    command = input('Invalid command.\nEnter a command: ')
    command = command.upper()
  code = command_to_code[command]
  return code

def get_value():
  value = int(input('Enter a value: '))
  while value < 0 or value > 255:
    value = input('Invalid value.\nEnter a value: ')
  return value

def need_value(code):
  c = code_to_command[code]
  if c is 'MOTOR' or c is 'STEER':
    return True
  #else
  return False

def to_byte(num):
  b = num.to_bytes(1, byteorder=ENDIAN, signed=False)
  return b

def write_code(code):
  command = code_to_command[code]
  print('Sent command: ', command)
  c = to_byte(code)
  port.write(c)

def write_value(value):
  print('Sent value: ', value)
  v = to_byte(value)
  print('Bytes: ', v)
  port.write(v)

def read_command():
  c = port.read(1)
  code = int.from_bytes(c, byteorder=ENDIAN, signed=False)
  if code not in code_to_command:
    #throw error
    code = 0
  command = code_to_command[code]
  return command

def read_value():
  v = port.read(1)
  value = int.from_bytes(v, byteorder=ENDIAN, signed=False)
  return value

def read_serial():
  n = port.in_waiting
  print('Number of messages in buffer: ', n)
  while n > 0:
    command = read_command()
    print('Received command: ', command)
    if command is 'MOTOR' or command is 'STEER' or command is 'STOP':
      value = read_value()
      print('Received value: ', value)
    n = n - 1

def main():
  # Reset buffers
  port.reset_input_buffer()
  port.reset_output_buffer()
  # Main loop
  while(True):
    code = get_code()
    write_code(code)
    if need_value(code):
      value = get_value()
      write_value(value)
    read_serial()

if __name__ == '__main__':
  main()
  # n > 0 read first code
  # if code is not OVER then it is an ERROR : "code received"
  # if code is OVER then resolve msg
  # if msg starts with MOTOR or STEER or STOP then report value

# loop:
#   ask for command
#     convert command to all caps
#     search dictionary for hex value
#   ask for value (if warranted)
#     if warranted, set byte value
#   execute command
#   read serial for debugging
#     if buffer has message, display it, else return to top of loop

