import math 

from controller import Robot, Camera, DistanceSensor
from vehicle import Driver


TIME_STEP = 32

robot = Robot()
#driver = Driver() 

rearCamera = robot.getDevice("rearCameraTest")
rearCamera.enable(TIME_STEP)


frontDistance = robot.getDevice("frontDistance")
frontDistance.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:

    rearFrame = rearCamera.getImage()
    frontDistanceVal = frontDistance.getValue()
    if frontDistanceVal > 400.0: 
        print("Front distance value is: " + str(frontDistanceVal))
        #speed = driver.getTargetCruisingSpeed()
        #driver.setCruisingSpeed(speed*.85)
    #print("Sensor value is: " + value)
    #print("Hello World!")


rearCamera.disable() 
frontDistance.enable() 
