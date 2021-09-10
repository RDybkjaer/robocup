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

ev3=EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)
claw_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE, [8,400])

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=34, axle_track=14)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

'''
grey: -5 10
white 30 55
black -45 -35
'''
claw_motor.run_time(30, 10000)
claw_motor.run(10000)
'''
def straightenUp (section, deviation):
    if section == 0 and 1 and 3:
        while deviation >= 30:
            robot.turn(45)
            deviation = line_sensor.reflection() - threshold
    if section == 2 and section >= 4:
        while deviation >= 30:
            robot.turn(-45)
            deviation = line_sensor.reflection() - threshold
        
       

section = 0
# Start following the line endlessly.
while section == 0:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold
    
    robot.drive(100,0)

    if deviation >=30: #White
        #straighten up
        straightenUp(section, deviation)
        #robot.turn(45)

        
    if deviation <= -35: #Black
        #new section
        section += 1
    #if deviation <= 30:
    #    ev3.speaker.beep()
    print(deviation)
    # You can wait for a short time or do other things in this loop.
    wait(10)

while section == 1:

'''
