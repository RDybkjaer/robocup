#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.B) #Left motor in port B
right_motor = Motor(Port.C) # Right motor in port C
claw_motor = Motor(Port.A) #Claw motor in port A
line_sensor = ColorSensor(Port.S2) #Line sensor in port
#push_sensor = TouchSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S1) #Gyro sensor in port 1

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=50, axle_track=138)




#grå 30 25 26    // 48 49
#hvid 52 46 65   // 86 87
#sort 6 9        // 9 8


GREY = 48
WHITE = 80
BLACK = 15

threshold = (GREY + WHITE) / 2



def follow_line(DRIVE_SPEED=150, P_GAIN=1.2,direction=1, breakable=0):
    i = 0
    #Drive_speed is in mm pr second
    
    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # For example, if the light value deviates from the threshold by 10, the robot
    # steers at 10*1.2 = 12 degrees per second.

    # Start following the line endlessly.

    while i == 0:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = P_GAIN * deviation * direction
    
        

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        if breakable == 1:
            break
    
        if line_sensor.reflection() < BLACK:
            i = 1

def switch_lane(NUMBER_TO_IGNORE=0, TURN_SIDE='RIGHT'): #BASIC CODE FOR CHANGING LANE - USED IN SEKVENS 1 & 4

    i=0

    """
    switch_lane is used to change lanes, by ignoring other lanes.
    Defined by TURN_SIDE, it'll turn either 45 degrees right, left or not at all.

    An idea might be to add at "forwards or backwards" argument, so it can function for sequence 5 as well.
    """

    robot.straight(50)
    #Following lines defines which way to turn. If no side is given, it'll beep and stop the function
    while True:
        if TURN_SIDE == 'RIGHT':
            turnRight(TURNANGLE = 45)
        elif TURN_SIDE == 'LEFT':
            turnLeft(TURNANGLE = 45)
        else:
            break
        robot.straight(50)
        robot.drive(150,0) #drive with 360deg/sec
        while line_sensor.reflection() > 60:
            wait(1)
        print('Out of the loop')
        """
        while i <= NUMBER_TO_IGNORE:
            print('I number to ignore løkke')
            last_value = line_sensor.reflection() - 20
            print('last value:')
            print(last_value)
            print('line senser:')
            print(line_sensor.reflection())

            if last_value > line_sensor.reflection():
                i = i + 1
                print('i stiger')
        print('ude af i løkke')
        """
        robot.stop()
        print('stopped ok')

        robot.straight(80)
        print('3.2cm')

        
        if TURN_SIDE == 'RIGHT':
            print('correction left')
            turnLeft(TURNANGLE = 45)
            #turnLeftToLine()
        elif TURN_SIDE == 'LEFT':
            print('correcting right')
            turnRight(TURNANGLE = 45)
            #turnRightToLine()
        else:
            print('not correcting')
        
        print('breaking')
        break
    
def pick_up():
    claw_motor.run_until_stalled(100, then=stop.HOLD, duty_limit=50)

def sekvens1():
    switch_lane(TURN_SIDE='RIGHT')
    follow_line()

def sekvens2():
    switch_lane(TURN_SIDE='LEFT')
    follow_line(DRIVE_SPEED=150, P_GAIN=2.5)

    """
    if line_sensor.reflection() <60:
        turnrate=2
    elif line_sensor.reflection() > 60:
        turnrate=1
    follow_line(DRIVE_SPEED=150,P_GAIN=turnrate,)
    """
    print('finished seq 2')

def sekvens3():
    
    '''
    The sequence used by the robot to finish task 3, the waterbottle task
    '''

    robot.stop()
    gyro_sensor.reset_angle(0)
    sensorVar = ultra_sensor.distance()
    print(sensorVar)
    #Following lines turn the robot untill it can "see" the bottle
    """
    while sensorVar > 500:
        print(sensorVar)
        robot.drive(0,30)
        sensorVar = ultra_sensor.distance()
    """
    turnRight(TURNANGLE=45)
    robot.straight(50)
    while line_sensor.reflection() > 55:
        robot.drive(150,0)
    while line_sensor.reflection() < 55:
        robot.drive(150,0)
    gyro_sensor.reset_angle(0)
    sensorVar_low = 500
    angle = 0
    for angle in range(0,75,5)
        sensorVar=ultra_sensor.distance()
        if sensorVar < sensorVar_low:
            sensorVar_low = sensorVar
            angle_low = angle
        turnRight(TURNANGLE=5)
        print('angle: ')
        print(angle)
        print('sensor: ')
        print(sensorVar)
        print('low: ')
        print(anglelow)

    turnLeft(TURNANGLE=75-angle_low)    
    
    
        
    approch_bottle()
    
    
    robot.stop()

    

    #Following lines drives the robot towards the bottle, slowing down as it approches
    """
    claw_motor.run_angle(speed=360, target_angle=360*lift_rotations)
    
    #Following lines drives towards the black line
    
    deviaition = line_sensor.reflection()

    while deviation > BLACK:
        robot.drive(100)
        deviaition = line_sensor.reflection()
    robot.stop()

    claw_motor.run_angle(speed=360, target_angle=-360*lift_rotations)

    robot.straight(-100)

    approch_bottle()

    claw_motor.run_angle(speed=360, target_angle=360*lift_rotations)

    robot.turn(-(gyro_sensor.angle()+90))

    """

def approch_bottle():
    while sensorVar > 50:
        drive_speed_bottle = (sensorVar * 2) / 2
        robot.drive(drive_speed_bottle)
        sensorVar = ultra_sensor.distance()

def loop(): #MAIN CODE - THIS CODE CALLS THE SUBFUNCTIONS
    follow_line()
    for sekvens in range(1,13):
        print(sekvens)

        if sekvens == 1:
            sekvens1()  

        elif sekvens == 2:
            sekvens2()        

        elif sekvens == 3:
            sekvens3()         

        elif sekvens == 4:
            sekvens4()            

        elif sekvens == 5:
            sekvens5()

        elif sekvens == 6:
            sekvens6()

        elif sekvens == 7:
            sekvens7()

        elif sekvens == 8:
            sekvens6()

        elif sekvens == 9:
            sekvens9()
        
        else:
            ev3.speaker.play_file(SoundFile.FANFARE)

def turnRight(ROTATION_SPEED = 45, TURNANGLE = 90):
    ang = 0
    gyro_sensor.reset_angle(0)
    robot.drive(0,ROTATION_SPEED)

    while  ang < TURNANGLE:
        ang = gyro_sensor.angle()

    robot.stop()

def turnLeft(ROTATION_SPEED = 45, TURNANGLE = 90):
    ang = 0
    gyro_sensor.reset_angle(0)
    robot.drive(0,-ROTATION_SPEED)

    while ang > -TURNANGLE:
        ang = gyro_sensor.angle()
    robot.stop()

def turnLeftToLine(ROTATION_SPEED = 45):
    robot.drive(0,-ROTATION_SPEED)
    while line_sensor.reflection() < 60:
        wait(1)
    robot.stop()

def turnRightToLine(ROTATION_SPEED = 45):
    robot.drive(0,ROTATION_SPEED)
    while line_sensor.reflection() < 60:
        wait(1)
    robot.stop()

def grab(speed, rotations)
    print('something')

def line_test():
    while True:
        deviation = line_sensor.reflection()
        print(deviation)

def angle_test():
    for i in range(0,180,15):
        gyro_sensor.reset_angle(0)
        robot.turn(i)
        ang = gyro_sensor.angle()

def ultra_test():
    while True:
        sensorVar = ultra_sensor.distance()
        print(sensorVar)

def test_loop():
    follow_line()
    sekvens3()


#loop()

test_loop()

#ultra_test()

#angle_test()

#line_test()

#turnLeft()     

#follow_line(DRIVE_SPEED=150, P_GAIN=2.5, direction=-1)
