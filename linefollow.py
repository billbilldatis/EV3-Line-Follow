#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
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
def calc_speed(minapostasi,maxspeed):
    distance = UltrasonicSensor(Port.S2)
    if distance < minapostasi:
        finalspeed = (maxspeed  *((100/ minapostasi * distance)))/100
    else:
        finalspeed =  maxspeed
    return finalspeed
def line_follow(speed,kp,kd,mode,angle,check):
    global error,line_sensor,der,last_error,left_motor,right_motor
    left_motor.reset_angle(0)
    StopWatch.reset()
    if check == True:
        if mode == 0:
            while angle < left_motor.angle():
                line_sensor[1] = ColorSensor(Port.S2)
                line_sensor[0] = ColorSensor(Port.S3)
                # Calculate the error from the threshold.
                error = line_sensor[0].reflection() - line_sensor[1].reflection()
                # Calculate the turn rate.
                der = last_error - error
                turn_rate = kp * error + (kd*last_error)
                speeda = speed - turn_rate
                speedb = speed + turn_rate
                # Set the drive base speed and turn rate.
                left_motor.dc(speeda)
                right_motor.dc(speedb)
                last_error = error
        if mode == 1:
            while line_sensor[1] < 25 and line_sensor[0] < 25:
                left_motor = Motor(Port.B)
                line_sensor[1] = ColorSensor(Port.S2)
                line_sensor[0] = ColorSensor(Port.S3)
                # Calculate the error from the threshold.
                error = line_sensor[0].reflection() - line_sensor[1].reflection()
                # Calculate the turn rate.
                der = last_error - error
                turn_rate = kp * error + (kd*last_error)
                speeda = speed - turn_rate
                speedb = speed + turn_rate
                # Set the drive base speed and turn rate.
                left_motor.dc(speeda)
                right_motor.dc(speedb)
                last_error = error
        if mode == 2:
            while timer < deg:
                timer = StopWatch.time()
                line_sensor[1] = ColorSensor(Port.S2)
                line_sensor[0] = ColorSensor(Port.S3)
                # Calculate the error from the threshold.
                error = line_sensor[0].reflection() - line_sensor[1].reflection()
                # Calculate the turn rate.
                der = last_error - error
                turn_rate = kp * error + (kd*last_error)
                speeda = speed - turn_rate
                speedb = speed + turn_rate
                # Set the drive base speed and turn rate.
                left_motor.dc(speeda)
                right_motor.dc(speedb)
                last_error = error
    else:
        if mode == 0:
            while angle < left_motor.angle():
                line_sensor[1] = ColorSensor(Port.S2)
                line_sensor[0] = ColorSensor(Port.S3)
                # Calculate the error from the threshold.
                error = line_sensor[0].reflection() - line_sensor[1].reflection()
                # Calculate the turn rate.
                der = last_error - error
                turn_rate = kp * error + (kd*last_error)
                speeda = calc_speed(15,speed) - turn_rate
                speedb = calc_speed(15,speed) + turn_rate
                # Set the drive base speed and turn rate.
                left_motor.dc(speeda)
                right_motor.dc(speedb)
                last_error = error
        if mode == 1:
            while line_sensor[1] < 25 and line_sensor[0] < 25:
                left_motor = Motor(Port.B)
                line_sensor[1] = ColorSensor(Port.S2)
                line_sensor[0] = ColorSensor(Port.S3)
                # Calculate the error from the threshold.
                error = line_sensor[0].reflection() - line_sensor[1].reflection()
                # Calculate the turn rate.
                der = last_error - error
                turn_rate = kp * error + (kd*last_error)
                speeda = calc_speed(15,speed) - turn_rate
                speedb = calc_speed(15,speed) + turn_rate
                # Set the drive base speed and turn rate.
                left_motor.dc(speeda)
                right_motor.dc(speedb)
                last_error = error
        if mode == 2:
            while timer < deg:
                timer = StopWatch.time()
                line_sensor[1] = ColorSensor(Port.S2)
                line_sensor[0] = ColorSensor(Port.S3)
                # Calculate the error from the threshold.
                error = line_sensor[0].reflection() - line_sensor[1].reflection()
                # Calculate the turn rate.
                der = last_error - error
                turn_rate = kp * error + (kd*last_error)
                speeda = calc_speed(15,speed) - turn_rate
                speedb = calc_speed(15,speed) + turn_rate
                # Set the drive base speed and turn rate.
                left_motor.dc(speeda)
                right_motor.dc(speedb)
                last_error = error
line_follow(100,0.4,4,0,200,False)
