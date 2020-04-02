# File: serial-test.py
# Desc: Serial command parser for executing car commands via the terminal.
# Copyright (c) 2020 Adam Peterson - All rights reserved

import serial
import time

# Constants
BAUD_RATE = 9600
ENDIAN = 'little'
SERIAL_PATH = '/dev/ttyACM1'

# Commands
command_to_code = {'ERROR':0, 'HELLO':1, 'MOTOR':2, 'OVER':3, 'STEER':4, 'STOP':5, 'CONNECTED':6}
code_to_command = {0:'ERROR', 1:'HELLO', 2:'MOTOR', 3:'OVER', 4:'STEER', 5:'STOP', 6:'CONNECTED'}

# Serial
port = serial.Serial(SERIAL_PATH, BAUD_RATE)
port.timeout = 1

def get_code():
  comm = input('Enter a command: ')
  comm = comm.upper()
  while comm not in command_to_code:
    comm = input('Invalid command.\nEnter a command: ')
    comm = comm.upper()
  code = command_to_code[comm]
  return code

def need_value(code):
  comm = code_to_command[code]
  if comm is 'MOTOR' or comm is 'STEER':
    return True
  #else
  return False

def get_value():
  val = int(input('Enter a value: '))
  while val < 0 or val > 255: #signed byte
    val = input('Invalid value.\nEnter a value: ')
  return val

def write_serial(msg):
  m = bytes(msg)
  port.write(m)

def read_command():
  code = get_byte()
  if code not in code_to_command:
    code = 0 #throw error
  command = code_to_command[code]
  return command

def get_byte():
  b = port.read(1)
  return int.from_bytes(b, byteorder=ENDIAN, signed=False)

def read_value():
  return get_byte()

def read_serial():
  print('Incoming messages:')
  while port.in_waiting > 0:
    comm = read_command()
    print('Received command: ', comm)
    if comm in ['MOTOR', 'STEER', 'STOP']:
      val = read_value()
      print('Received value: ', val)

def main():
  # Establish connection
  connected = False
  while(not connected):
    print('Connecting...')
    write_serial(command_to_code['HELLO'])
    shake = read_command()
    if shake not in ['HELLO', 'CONNECTED']:
      time.sleep(1)
      continue
    else:
      connected = True
  # Reset buffers
    # port.reset_input_buffer()
    # port.reset_output_buffer()
  # Main loop
  while(connected):
    msg = []
    code = get_code()
    msg.append(code)
    if need_value(code):
      val = get_value()
      msg.append(val)
    write_serial(msg)
    read_serial()

if __name__ == '__main__':
  main()
