# File: controller.py
# Desc: Main controller module for the car.
# Copyright (c) 2020 Adam Peterson

import serial
import time
import params
from control import Command, read_byte, read_command
from autodriver.models.exceptions import ConnectionError

# Init serial
port = serial.Serial(params.SERIAL_PATH, params.BAUD_RATE)

if params.DEBUG:
    port.timeout = None
else:
    port.timeout = 1


def execute_commands(c, v):
    """
    Executes a control command with an associated value.

    @param c: The Command object representing the command to be executed.
    @param v: The integer value.
    """
    msg = [c.value, v]
    port.write(msg)


def execute_command(c):
    """
    Executes a single control command.

    @param c: The Command object representing the command to be executed.
    """
    port.write([c.value])


def motor(p):
    """
    Sets the motor to a percentage (0.0, 1.0) of its maximum speed.

    @param p: A float representing the desired percentage.
    """
    threshold = params.MOTOR_MIN
    speed = int(p * params.MOTOR_MAX)
    if speed < threshold:
        speed = 0
    execute_commands(Command.MOTOR, speed)


def steer(a):
    """
    Sets the steering servo to a desired angle within its range.

    @param a: An integer represening the angle to be set.
    """
    if a < params.STEERING_MIN:
        angle = params.STEERING_MIN
    elif a > params.STEERING_MAX:
        angle = params.STEERING_MAX
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
    for a in range(params.STEERING_MIN, params.STEERING_MAX, 1):
        print(a)
        steer(a)
        time.sleep(0.1)
    for a in range(params.STEERING_MAX, params.STEERING_MIN, -1):
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
    while not connected and attempt < 10:
        if port.in_waiting > 0:
            shake = read_command(port)
            if shake is Command.OVER:
                connected = True
                print('Connected!')
        else:
            attempt += 1
            time.sleep(0.5)
            execute_command(Command.HELLO)
            print('...')
    # Delay and clear everything in the buffer.
    time.sleep(5)
    port.reset_input_buffer()
    if not connected:
        raise ConnectionError('Unable to initialize serial communication')


def read_serial():  # TODO account for 'ERROR'
    """
    Reads a command from serial and prints debugging output to the console. Ignores 'OVER' and 'HELLO'.
    """
    needs_val = (Command.MOTOR, Command.STEER, Command.REVERSE)
    command = read_command(port)
    while (command is not Command.OVER):
        if params.DEBUG:
            if command in needs_val:
                val = read_byte(port)
                print('%s %d' % (command, val))
            else:
                print('%s' % (command))
        command = read_command(port)
