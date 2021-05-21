#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
line_sensor = [0,0]
# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
last_error = 0
der = 0
# Initialize the color sensor.
line_sensor[0] = ColorSensor(Port.S3)
line_sensor[1] = ColorSensor(Port.S2)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the drive base.
# Start following the line endlessly.
def line_follow(DRIVE_SPEED,kp,kd):
    global error,line_sensor,der,last_error,left_motor,right_motor
    line_sensor[1] = ColorSensor(Port.S2)
    line_sensor[0] = ColorSensor(Port.S3)
    # Calculate the error from the threshold.
    error = line_sensor[0].reflection() - line_sensor[1].reflection()
    # Calculate the turn rate.
    der = last_error - error
    turn_rate = kp * error + (kd*last_error)
    speeda = DRIVE_SPEED - turn_rate
    speedb = DRIVE_SPEED + turn_rate
    # Set the drive base speed and turn rate.
    left_motor.dc(speeda)
    right_motor.dc(speedb)
    last_error = error
line_follow(100,0.4,4)
