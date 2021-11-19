import math
import numpy as np

##############################
#   utils
#############################
def get_bearing_in_rad(compass):
  north = compass.getValues()
  rad = math.atan2(north[0], north[2]);
  bearing = (rad - 1.5708) / math.pi * 180.0;
  if (bearing < 0.0):
    bearing = bearing + 360.0

  return math.radians(bearing)


def get_current_state(gps,compass,driver,b):
    angle = driver.getSteeringAngle()
    values = gps.getValues()
    psi = get_bearing_in_rad(compass)
    spd = gps.getSpeed()
    #spd = driver.getCurrentSpeed()/3.6
    state = [values[0],values[2],psi,spd,b]

    return state


def get_back_vehicle_distance(radar_b, v_state):
    ####################################
    # process the back radar
    #####################################

    n_targets =  radar_b.getNumberOfTargets()
    targets = radar_b.getTargets()

    lag_vehicle_spd = 22.00
    lag_vehicle_dist = 60

    blind_spot = False
    is_lag = False

    for i in range(n_targets):
        dist =  targets[i].distance
        speed = targets[i].speed
        azi = targets[i].azimuth
        K = dist*azi
        ##print("K",K)

        #print ('Back Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6), ' theta:', targets[i].azimuth )

        if(K>0 and K<5):
            if(dist<lag_vehicle_dist):
              lag_vehicle_dist = dist
              lag_vehicle_spd = -1*speed + v_state[3]
              ##print("lag_vehicle_dist","lag_vehicle_spd",lag_vehicle_dist,lag_vehicle_spd)
              #lag_veh_profile(lag_vehicle_dist,lag_vehicle_spd)   #read
              is_lag = True

    return lag_vehicle_dist, lag_vehicle_spd


def get_front_vehicle_distance(radar_f,v_state):

  n_targets =  radar_f.getNumberOfTargets()
  targets = radar_f.getTargets()

  lead_vehicle_spd = 22.00
  lead_vehicle_dist = 60

  for i in range(n_targets):
      dist =  targets[i].distance
      speed = targets[i].speed
      azi = targets[i].azimuth
      K = dist*azi

      #print ('Front Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6), ' theta:', targets[i].azimuth )

      if(K > -3.5 and K < -2): # and dist>2 and dist<GAP_LAG):
          if(dist<lead_vehicle_dist):
              lead_vehicle_dist = dist
              lead_vehicle_spd = speed + v_state[3]
              is_lead = True

  return lead_vehicle_dist, lead_vehicle_spd
