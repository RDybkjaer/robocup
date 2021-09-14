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
claw_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE) #Claw motor in port A
line_sensor = ColorSensor(Port.S3) #Line sensor in port 3
push_sensor = TouchSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=34, axle_track=14)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

'''
grey: -5 10
white 30 55
black -45 -35
'''

#DER BØR TILFØJES EN VARIABEL SOM DEFINERER HVILKEN SIDE DEN SKAL TJEKKE TIL FØRST. FX EFTER SEKVENS2 SKAL DEN TIL HØJRE,
#OG EFTER SEKVENS3 SKAL DEN TIL VENSTRE


def loop: #MAIN CODE - THIS CODE CALLS THE SUBFUNCTIONS
    for sekvens in range(1,10)
        autoDrive(DIST=100)

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


def sekvens1(): #CHANGE LANE TWO DIRECTIONS - READY FOR TEST
    '''
    Function for sekvens 1, changing line twice.
    '''
    turn_drive_ignore(0, 'RIGHT')
    autoDrive(DIST=100)
    turn_drive_ignore(0, 'LEFT')

def sekvens2(): #PICK UP WATERBOTTLE - READY FOR TEST
    '''
    The sequence used by the robot to finish task 3, the waterbottle task
    '''

    #Following lines drives towards the bottle and picks it up
    sensorVar = ultra_sensor.distance()
    robot.turn(45)
    while sensorVar > 50:
        autoDrive(BREAKABLE=1,DIST=50)
        sensorVar = ultra_sensor.distance()
    claw_motor.run_time(360, 5000)
    
    #Following lines drives towards the black line
    deviation = deviationCheck()
    while deviation >= -35:
        robot.drive(100)
    robot.stop()

    #Folloing lines puts down the bottle and return back to the track
    claw_motor.runtime(-360, 5000)
    robot.straigt(100)
    robot.turn(-145)

    while deviation >= 35:
        robot.drive(100)
    robot.stop()

def sekvens3(): #SEESAW - UNFINISHED
    '''
    Function to do task 3, the seesaw.
    An idea might be to put on a gyroscope and make to robot stop when the movement happens?
    '''
    robot.turn(45)
    robot.straight(100)
    autoDrive(DIST=100)
    autoDrive(DIST=10)

def sekvens4(): #CHANGE LANE TWICE - READY FOR TEST
    autoDrive(DIST=10)
    turn_drive_ignore(1, 'LEFT')

def sekvens5(): #UNFINISHED
    '''
    Ingen idé so far.
    Drej til venstre
    Fortsæt frem til stregen
    Kør ind på midten vha robot.straight.
    Drej til du kan se flasken
    Saml flasken op - bør man lave en fællesfunktion for sekvens 2 og 5?
    Bak ud og tæl stregerne - turn_drive_ignore??
    Stil flasken
    Bak væk.
    Find ud.
    Drej til venstre ved udgangen
    '''

def sekvens6(): #USED BOTH FOR SEKVENS6 & 8 - UNFINISHED
    '''
    NOTHING TO SEE HERE
    '''

def sekvens7(): #UNFINISHED
    '''
    NOTHING TO SEE HERE
    '''

def sekvens9(): #UNFINISHED
    '''
    NOTHING TO SEE HERE
    '''





def turn_drive_ignore(NUMBER_TO_IGNORE, SIDE_TO_TURN) #BASIC CODE FOR CHANGING LANE - USED IN SEKVENS 1 & 4
    '''
    Turn_drive_ignore is used to change lanes, by ignoring other lanes.
    Defined by SIDE_TO_TURN, it'll turn either 45 degrees right, left or not at all.

    An idea might be to add at "forwards or backwards" argument, so it can function for sequence 5 as well.

    '''
    deviation = deviationCheck()
    
    #Following lines defines which way to turn. If no side is given, it'll beep and stop the function
    if SIDE_TO_TURN == 'RIGHT':
        turnAngle = 45
    elif SIDE_TO_TURN == 'LEFT':
        turnAngle = -45
    else:
        turnAngle = 0
    
    robot.turn(turnAngle):
    robot.drive(360): #drive with 360deg/sec

    while i < NUMBER_TO_IGNORE:
        if deviaition >= deviationcheck
            i = i+1
        
        deviation = deviaitionCheck:
    robot.stop()

    robot.turn(turnAngle * -1)
    break

def straightenUp (): #BASIC CODE TO KEEP ON THE LINE - USED IN AUTODRIVE
    '''
    straightenUp helps the robot return to the line.
    It turns 50 degrees in each direction and checks wether the line shows up. C destines wether it turns right or left.
    Returns the angle at which it found the line, to be used in later corrections
    '''
    deviation = deviationCheck()
    while deviation >= 30: #white
        c = 1
        for i in range(0,50*c,10*c):
            robot.turn(i):
            deviation = deviationCheck()
        robot.turn(-50)
        c = c * -1
    return i

def autoDrive(BREAKABLE=0, DIST=100): #BASECODE TO DRIVE WHILE ON THE LINE
    '''
    autoDrive is a function that drives the robot while looking for deviation to the line.
    It is supposed to have a distance driven added
    '''
    while deviaition <= -35: #black
        if deviation < 30:
            robot.straight(DISTANCE)
            deviation = deviationCheck()
        else:
            '''
            straightenUp guides the robot back on track, and returns the turning value it needed to do(correction).
            This allows us to center the robot on the line(5 cm to center?) and return to driving
            '''
            correction = straightenUp()
            robot.straight(50):
            robot.turn(correction)
            deviation = deviationCheck()
        
        if BREAKABLE == TRUE #Tells us wether to stop at a black line, or only drive the distance given in "distance"
            break
    
def deviationCheck (): # CHECKING DEVIATION. To be used in deviation = deviationCheck()
