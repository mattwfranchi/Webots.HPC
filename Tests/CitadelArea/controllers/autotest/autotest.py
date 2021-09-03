from controller import Robot, DistanceSensor, Radar

time = 32

voi = Robot()

voi_frontr = voi.getDevice("front_radar")
voi_frontr.enable(time)

voi_backr = voi.getDevice("back_radar")
voi_backr.enable(time)

voi_distl = voi.getDevice("ds_left")
voi_distl.enable(time)

voi_distr = voi.getDevice("ds_right")
voi_distr.enable(time)

while voi.step(time) != 1:
    print("Distance Sensor Values: Left: " + voi_distl.getValue() + " Right: " + voi_distr.getValue())
    
    autocar = 1
    
    print("Radar Vehicles Detected = Front: " + voi_frontr.getNumberOfTargets()-autocar + " Back: " + voi_backr.getNumberOfTargets()-autocar)


voi_frontr.disable()
voi_backr.disable()


