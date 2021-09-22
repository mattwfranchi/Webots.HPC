import math
import time
import json
from vehicle import Driver

from controller import GPS, Receiver, Lidar, Radar,RadarTarget


driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(30.0)
    

prev_t = driver.getTime()

radar = Radar("esr")
radar.enable(10)

gps = GPS("gps")
gps.enable(10)

def idm(v, d, del_v):
    a_norm = 4.57
    d_dist = 4
    v_max = 13 #m/s
    
    gamma = 4.0
    
    s0 = 2
    T = 0.5
    a = 4.57
    b = 1.7
    
    x = v*T + (v*del_v)/(2* math.sqrt(a*b))
    d_dist = s0 + max(0,x)
    
    acc = a * (1 - math.pow((spd/v_max), gamma) - math.pow((d_dist/d),2))
    
    return v + acc
     

t1 = driver.getTime()

   
while(driver.step() != -1):
    n_targets = radar.getNumberOfTargets()
    targets = radar.getTargets()
    

    #print ('N: ', n_targets, ' targets: ', targets)
    
    for i in range(n_targets):
        print (targets[i].distance, targets[i].azimuth)
        dist = targets[i].distance
        theta = targets[i].azimuth
        spd =  targets[i].speed
        
        my_spd = gps.getSpeed()
        
        del_v = my_spd - spd
        
        if(math.fabs(dist*theta)<1.0):
            a = idm(my_spd, dist,del_v)
            
            t2= driver.getTime()
            
            print ('target speed : ', spd*3.6, ' my spd : ', my_spd*3.6, 'dist:', dist)
            
            print ('acc : ', a  , ' del time : ', (t2-t1))
            
            v = spd+ a
            
            #print ()
            driver.setCruisingSpeed(v*3.6)
            t1 = t2
            
        else:
            print ('outside lane')
    