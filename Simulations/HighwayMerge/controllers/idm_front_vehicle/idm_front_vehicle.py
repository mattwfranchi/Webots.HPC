import math
import time
import json
from vehicle import Driver


driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(20.0)
    

prev_t = driver.getTime()

while(driver.step() != -1):
    pass

