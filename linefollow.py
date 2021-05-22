#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor , UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
line_sensor = ColorSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
kd = 2
kp = 1.2
ls_error = 0
def calc_speed(minapostasi,maxspeed):
    distance = UltrasonicSensor(Port.S2)
    if distance < minapostasi:
        finalspeed = (maxspeed  *((100/ minapostasi * distance)))/100
    else:
        finalspeed =  maxspeed
    return finalspeed
def linefollowcode(speed,kp,kd):
    global ls_error,left_motor,right_motor
    # Calculate the deviation from the threshold.
    error = line_sensor.reflection() - ColorSensor(Port.S2).reflection()
    deriv = error - ls_error

    # Calculate the turn rate.
    turn_rate = kp * error + kd * deriv

    # Set the drive base speed and turn rate.
    if speed-turn_rate < 100:
        try:
            left_motor.dc(speed-turn_rate)
        except:
            left_motor.dc(speed*-1)
    else:
        left_motor.dc(speed)
    if speed+turn_rate < 100:
        try:
            right_motor.dc(speed+turn_rate)
        except:
            right_motor.dc(speed*-1)
    else:
        right_motor.dc(speed)
    ls_error = error
left_motor.reset_angle(0)
right_motor.reset_angle(0)
def line_follow(speed,kp,kd,mode,angle,check):
    timer = 0
    global error,line_sensor,der,last_error,left_motor,right_motor
    left_motor.reset_angle(0)
    #StopWatch.reset()
    if check == True:
        if mode == 0:
            while angle < left_motor.angle():
                linefollowcode(speed,kp,kd)
        if mode == 1:
            while not( line_sensor.reflection() > 25 and ColorSensor(Port.S2).reflection() > 25):
                linefollowcode(speed,kp,kd)
        if mode == 2:
            while timer < deg:
                #timer = StopWatch.time()
                linefollowcode(speed,kp,kd)
    else:
        if mode == 0:
            while angle < left_motor.angle():
                linefollowcode(speed,kp,kd)
        if mode == 1:
            while not( line_sensor.reflection() > 25 and ColorSensor(Port.S2).reflection() > 25):
                linefollowcode(speed,kp,kd)
        if mode == 2:
            while timer < deg:
                #linefollowcode(speed,kp,kd)
                timer = StopWatch.time()
left_motor.reset_angle(0)
right_motor.reset_angle(0)
right_motor.hold()
left_motor.hold()
