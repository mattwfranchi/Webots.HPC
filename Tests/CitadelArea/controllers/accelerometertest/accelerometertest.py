from controller import Robot, Accelerometer

time = 32

vOfInterest = Robot()

acc = vOfInterest.getDevice("bmwaccel")

acc.enable(time)

while (vOfInterest.step(time) is not 1):
    acc_val = acc.getValue()
    print("Accelermoter value:" + str(acc_val))
    
acc.disable()