# File: controller.py
# Desc: Main controller module for the car.
# Copyright (c) 2020 Adam Peterson - All rights reserved

import serial
import time
import threading
from parameters import *
from control import Command, read_byte, read_command, write_serial, parse_command

# Constants
SERIAL_PATH = '/dev/ttyACM0'

# Init serial
port = serial.Serial(SERIAL_PATH, BAUD_RATE)
port.timeout = None
port.flush()


def execute_commands(c, v):
    """
    Executes a control command with an associated value.
  
    @param c: The Command object representing the command to be executed.
    @param v: The integer value.
    """
<<<<<<< HEAD
    msg = [c, v]
    write_serial(port, msg)


def execute_command(c):
    """
    Executes a single control command.
  
    @param c: The Command object representing the command to be executed.
    """
    write_serial(port, [c])


def motor(p):
    """
    Sets the motor to a percentage (0.0, 1.0) of its maximum speed.
  
    @param p: A float representing the desired percentage.
    """
    threshold = MOTOR_MIN
    speed = int(p * MOTOR_MAX)
    if speed < threshold:
        speed = 0
    execute_commands(Command.MOTOR.value, speed)


def steer(a):
    """
    Sets the steering servo to a desired angle within its range.
  
    @param a: An integer represening the angle to be set.
    """
    if a < STEERING_MIN:
        angle = STEERING_MIN
    elif a > STEERING_MAX:
        angle = STEERING_MAX
    else:
        angle = a
    execute_commands(Command.STEER.value, angle)


def reverse(r):
    """
    Reverses the direction of the motor. Stops the motor before changing direction.
  
    @param r: A bool indicating the motor should reverse.
    """
    if r:
        rev = 1
    else:
        rev = 0
    stop()
    execute_commands(Command.REVERSE.value, rev)


def stop():
    """Stops the motor."""
    execute_commands(Command.MOTOR.value, 0)


def reset():
    """Stops the motor and resets it to its center position."""
    steer(90)
    stop()


def demo():
    def run_motor():
        for p in range(0, 100, 10):
            per = p / 100
            print(p, '%')
            motor(per)
            time.sleep(0.25)
            continue
        for p in range(100, 0, -10):
            per = p / 100
            print(p, '%')
            motor(per)
            time.sleep(0.25)
            continue

    print('Running demo...')
    print('Steering at angle:')
    for a in range(STEERING_MIN, STEERING_MAX, 1):
        print(a)
        steer(a)
        time.sleep(0.1)
        continue
    for a in range(STEERING_MAX, STEERING_MIN, -1):
        print(a)
        steer(a)
        time.sleep(0.1)
        continue
    print('Returning steering to center...')
    steer(90)
    print('Forward motor...')
    reverse(False)
    run_motor()
    print('Reverse motor...')
    reverse(True)
    run_motor()
    reset()
    print('End of demo.')


def connect():
    """Attempts to establish serial connection with Arduino."""
    connected = False
    attempt = 0
    while (not connected and attempt < 5):
        print('Connecting...')
        execute_command(Command.HELLO.value)
        shake = read_byte(port)
        if shake in [Command.HELLO.value, Command.OVER.value]:
            connected = True
            print('Connection Successful')
        else:
            time.sleep(1)
            attempt += 1

    if not connected:
        print('Connection Failed')
    return connected


def read_serial(s):
    """
    Daemon thread function enabling non-blocking reads

    @param s: Serial object to be read.
    """
    while True:
        command = read_command(port)
        val = read_byte(port)
        print('%s %d' % (command, val))


def main():
    if not connect():
        return

    # non blocking read loop in another thread
    r_thread = threading.Thread(target=read_serial, args=(port, ))
    r_thread.daemon = True
    r_thread.start()

    cmd_menu = '''Commands: 
    set motor speed percent --- p <val>
    set steer angle --- a <val>
    reverse --- r <val>
    end session --- exit
    print menu -- cmds
    '''
    print(cmd_menu)
    exit = False
    while not exit:
        args = input('Enter command: ').split()
        if len(args) == 1:
            if args[0] == 'cmds':
                print(cmd_menu)
            elif args[0] == 'exit':
                exit = True
        elif len(args) == 2:
            if args[0] == 'p':
                motor(args[1])
            elif args[0] == 'a':
                steer(args[1])
            elif args[0] == 'r':
                reverse(args[1])
        # time.sleep(0.3)


if __name__ == '__main__':
    main()
