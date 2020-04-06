# File: controller.py
# Desc: Main controller module for the car.
# Copyright (c) 2020 Adam Peterson

import serial
import time
import threading
from parameters import *
from control import Command, read_byte, read_command, write_serial

# Constants
SERIAL_PATH = '/dev/ttyACM0'

# Init serial
port = serial.Serial(SERIAL_PATH, BAUD_RATE)
port.timeout = None


def execute_commands(c, v):
    """
    Executes a control command with an associated value.
  
    @param c: The Command object representing the command to be executed.
    @param v: The integer value.
    """
    msg = [c.value, v]
    write_serial(port, msg)


def execute_command(c):
    """
    Executes a single control command.
  
    @param c: The Command object representing the command to be executed.
    """
    write_serial(port, [c.value])


def motor(p):
    """
    Sets the motor to a percentage (0.0, 1.0) of its maximum speed.
  
    @param p: A float representing the desired percentage.
    """
    threshold = MOTOR_MIN
    speed = int(p * MOTOR_MAX)
    if speed < threshold:
        speed = 0
    execute_commands(Command.MOTOR, speed)


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
    execute_commands(Command.STEER, angle)


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
    execute_commands(Command.REVERSE, rev)


def stop():
    """Stops the motor."""
    execute_commands(Command.MOTOR, 0)


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
        for p in range(100, 0, -10):
            per = p / 100
            print(p, '%')
            motor(per)
            time.sleep(0.25)

    print('Running demo...')
    print('Steering at angle:')
    for a in range(STEERING_MIN, STEERING_MAX, 1):
        print(a)
        steer(a)
        time.sleep(0.1)
    for a in range(STEERING_MAX, STEERING_MIN, -1):
        print(a)
        steer(a)
        time.sleep(0.1)
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
    print('Connecting with Arduino...')
    # See if we're already connected.
    execute_command(Command.OVER)
    shake = read_command(port)
    if shake is Command.OVER:
        connected = True
        print('Already connected.')
    # Otherwise, attempt to connect.
    while (not connected and attempt < 5):
        shake = read_command(port)
        if shake is Command.HELLO:
            execute_command(Command.HELLO)
            print('...')
        elif shake is Command.OVER:
            connected = True
            print('Connected!')
        else:
            time.sleep(1)
            attempt += 1
    if not connected:
        print('Unable to connect.')
    # Delay and clear everything in the buffer.
    time.sleep(5)
    port.reset_input_buffer()
    return connected


def read_serial(s):
    """
    Daemon thread function enabling non-blocking reads

    @param s: Serial object to be read.
    """
    needs_val = (Command.MOTOR, Command.STEER, Command.REVERSE)
    command = read_command(s)
    while (command is not Command.OVER):
        if DEBUG:
            if command in needs_val:
                val = read_bytes(s)
                print('%s %d' % (command, val))
            else:
                print('%s' % (command))
        command = read_command(s)
    print('%s' % (command)) # delete me, for debugging


def main():
    if not connect():
        return

    cmd_menu = '''Commands: 
    set motor speed percent --- p <val>
    set steer angle --- a <val>
    reverse --- r <0/1>
    end session --- exit
    print menu --- cmds
    '''
    print(cmd_menu)
    exit = False
    while not exit: #TODO Catch input errors, specifically ValueError for typos.
        args = input('Enter command: ').split()
        if len(args) == 1:
            if args[0] == 'cmds':
                print(cmd_menu)
            elif args[0] == 'exit':
                reset()
                exit = True
        elif len(args) == 2:
            if args[0] == 'p':
                motor(float(args[1]))
            elif args[0] == 'a':
                steer(int(args[1]))
            elif args[0] == 'r':
                reverse(bool(args[1]))
            read_serial(port)


if __name__ == '__main__':
    main()
