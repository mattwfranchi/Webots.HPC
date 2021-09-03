import math

from vehicle import Driver
#from controller import Camera

driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(20)

#camera = driver.getCamera("camera")
#camera.enable(10)
#camera.recognitionEnable(50)

while driver.step() != -1:
  #angle = 0.3 * math.cos(driver.getTime())
  #driver.setSteeringAngle(angle)
  #nofobj=camera.getRecognitionNumberOfObjects()
  #print(nofobj)








