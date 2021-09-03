###############################
# lane changing decision on gap based
###############################
def change_lane_on_gap(radar_b,radar_f,radar_l):
  GAP_LEAD = 54 
  GAP_LAG  = 52
  ####################################
  # process the back radar
  #####################################
  n_targets =  radar_b.getNumberOfTargets()
  targets = radar_b.getTargets()
  
  blind_spot = False
  for i in range(n_targets):
      dist =  targets[i].distance
      speed = targets[i].speed
      azi = targets[i].azimuth
      K = dist*azi
      
      #print ('Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6+driver.getCurrentSpeed()), ' theta:', targets[i].azimuth )
      
      if(K>0 and K<5 and dist<GAP_LAG):
          return False
          
  ####################################
  # process the front radar
  #####################################
  n_targets =  radar_f.getNumberOfTargets()
  targets = radar_f.getTargets()
  
  blind_spot = False
  for i in range(n_targets):
      dist =  targets[i].distance
      speed = targets[i].speed
      azi = targets[i].azimuth
      K = dist*azi
      
      #print ('Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6+driver.getCurrentSpeed()), ' theta:', targets[i].azimuth )
      
      if(K > -3.5 and K < -2 and dist<GAP_LEAD): # and dist>2 and dist<GAP_LAG):
          return False
          
  ####################################
  # process the bliendspot radar
  #####################################   
  n_targets =  radar_l.getNumberOfTargets()
  targets = radar_l.getTargets()
  for i in range(n_targets):
      dist =  targets[i].distance
      speed = targets[i].speed
      azi = targets[i].azimuth
      K = dist*azi
      
      if(dist>3.0 and dist<7):
          return False
   
  return True