"""py_rsu_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Emitter
import struct
import time
import json

# create the Robot instance.
robot = Robot()
emitter = Emitter("emitter")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

###############################################
# Collecct info from 

#################################################
file_path = "D:\\webot_out\\risk_based\\output.txt"

with open(file_path,mode='w') as file:
    file.close()
#######################################

def read_main_lane():
    file_path = "D:\\webot_out\\risk_based\\output.txt"
    file = open(file_path,mode='r')
    all_of_it = file.read()
    file.close()
    
    return all_of_it
    #for line in all_of_it.splitlines():
    #    loaded_json = json.loads(line)
    #    print (loaded_json)
    
   




# Main loop:
# - perform simulation steps until Webots is stopping the controller

count = 0
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #message = struct.pack("chd","a",45,120.08)
    
    ##  Step 1: Get the data of other vehicles near the mergeing area
    
    message = read_main_lane()
    

    ##  Step 2: Position, Speed, Accelration of each vehicle will be send to.
    
    if(len(message)>0):
        emitter.send(message.encode())
        

    count+=1
    #time.sleep(0.1)
    
    #print ("Emitter sending:", message)
    #pass

# Enter here exit cleanup code.
