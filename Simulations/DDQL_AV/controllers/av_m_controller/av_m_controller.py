import math
import time
import json
import random
from vehicle import Driver
from controller import Supervisor
from controller import Camera, GPS
from controller import Receiver, Lidar, Radar,RadarTarget, Compass, TouchSensor
import cv2
import numpy as np
import pandas as pd
from LaneKeeping import *
from haversine import haversine, Unit
from shapely.geometry import Point, Polygon
from mpc import *
from state import *
from lane_change import *
from lane_change_risk import *
import os, os.path
import time

from collections import deque
lane_change_q = deque(maxlen=3)
lead_veh_dist_tracking = deque(maxlen=4)
lead_veh_spd_tracking = deque(maxlen=4)

driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(0.0)


DEFAULT_OUT_SPEED = 22.00
DEFAULT_OUT_TTC = np.nan
DEFAULT_OUT_ACC = 0

# keeps track of target speed to reach if CAV has to slow down in order to
# complete merge
target_speed = 0

receiver = Receiver("receiver")
receiver.enable(50)

camera_l = Camera("camera_left")
camera_l.enable(50)

camera_r = Camera("camera_right")
camera_r.enable(50)

radar_b = Radar("esr_back")
radar_b.enable(50)

radar_l = Radar("esr_left")
radar_l.enable(50)

radar_f = Radar("esr_front")
radar_f.enable(50)

compass = Compass("compass")
compass.enable(50)
#camera.recognitionEnable(50)
count = 0

gps = GPS("gps")
gps.enable(50)

touch_front = TouchSensor("cav_touch_front")
touch_front.enable(50)

touch_side = TouchSensor("cav_touch_side")
touch_side.enable(50)

touch_rear = TouchSensor("cav_touch_rear")
touch_rear.enable(50)



#lidar = Lidar("lms291")
#lidar.enable(10)
#lidar.enablePointCloud()
# save trajectory data from SUMO

proj_cwd = os.getcwd()
proj_cwd = os.path.abspath(__file__+'/../../../../..')

#trajectory_profile_column_names = ["X-Coord","Z-Coord","Bearing","Speed","Current State"]
#trajectory_profile_df = pd.DataFrame(columns=trajectory_profile_column_names)

#risk_profile_column_names = ["X-Coord","Z-Coord","Bearing","Speed","Current State"]
#risk_profile_df = pd.DataFrame(columns=risk_profile_column_names)

risk_data_column_names = ["Speed_Merge", "Acc_Merge", "Steering_Merge", "Speed_Lead", "Acc_Lead",
"Gap_Lead", "Speed_Lag","Acce_Lag","Gap_Lag", "Remaining_Distance",
 "Num_Adj_Vehs_Start", "Num_Adj_Vehs_End", "TTCf", "TTCb", "Risk_Status","acce_change"]
risk_data_df = pd.DataFrame(columns=risk_data_column_names)

adjacent_cars_column_names = ["Distance","Received Power","Speed","Azimuth"]
adjacent_cars_start_df = pd.DataFrame(columns=adjacent_cars_column_names)
adjacent_cars_merge_df = pd.DataFrame(columns=adjacent_cars_column_names)













def sigmoid(x):
    return (1/(1+math.exp(-x)))

def del_sig(x):
    return sigmoid(x)*(1-sigmoid(x))

# Based on states then overrides to show current state #
LANE_KEEP = 0
MERGINING = 1
PERFORMED_MERGING = 2
current_state = LANE_KEEP
CUR_STATE = ["MERGINING","LANE_KEEP", "MERGING"]
NORMAL_SPD = 104

# Determines when the autonomous vehicle begins to move #
TIME_DEF = random.randint(200,450) / 10


########################################################

is_front = False
is_back = False
stop_lane_change = False
start_lane_change = False

# FLAGS FOR SNAPSHOTS
merge_point_reached = False
start_point_reached = False


vehicle_speeds = {}

def get_vehicle_speed_by_id(vid):
    global vehicle_speeds
    if(vid in vehicle_speeds):
       return vehicle_speeds[vid]


############################################################
#              IDM Model
############################################################
def idm(v, d, del_v):
    a_norm = 4.7
    d_dist = 2
    v_max = 22 #m/s

    gamma = 4.0

    s0 = 2
    T = 1.5
    a = 4.73
    b = 1.7

    x = max(0,v*T + (v*del_v)/(2* math.sqrt(a*b)))
    d_dist = s0 + x

    acc = a * (1 - math.pow((v/v_max), gamma) - math.pow((d_dist/d),2))

    return v + 0.04*acc

###############################
#
###############################



risk_lane_change = False




leader_spd = -1
leader_dist = 1000.0

count = 1000

last_time = driver.getTime()
cur_time = last_time
last_error = 0.0
target = -0.65
kp = 0.6
ki = 0.01
kd = 0.01

global err_sum
err_sum = float(0.0)


def distance(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2+ (p1[1]-p2[1])**2)


prev_t = driver.getTime()
timestamp = str(int(time.time()))
random_file_flag = str(random.randint(0,1000000))


# Output variables to write to csv
speed_merge = 0
acce_merge = 0
steering_merge = 0
speed_lead = 0
acce_lead = 0
gap_lead = 0
speed_lag = 0
acce_lag = 0
gap_lag = 0
remaining_dist = 0
num_vehs_adj_start = 0
num_vehs_adj_merge = 0
risk_status = 1
acce_change = 0

COLLISION_FLAG = 0

def collision_check():
    global COLLISION_FLAG
    #COLLISION_FLAG = 0
    if(COLLISION_FLAG == 0):
        COLLISION_FLAG = (touch_front.getValue() + touch_rear.getValue() + touch_side.getValue()) > 0

##################################
#      Delay Routine
# Collect variables at each time stamp
##################################
while(driver.step() != -1):
    t = driver.getTime()


    if(t- prev_t>TIME_DEF):
        break
    elif (start_point_reached is False):
        veh_count = 1
        for target in radar_f.getTargets():
            #adjacent_cars_start_df.loc[veh_count] = [target.distance, target.received_power, target.speed, target.azimuth]
            veh_count += 1
        for target in radar_l.getTargets():
            #adjacent_cars_start_df.loc[veh_count] = [target.distance, target.received_power, target.speed, target.azimuth]
            veh_count += 1
        for target in radar_b.getTargets():
            #adjacent_cars_start_df.loc[veh_count] = [target.distance, target.received_power, target.speed, target.azimuth]
            veh_count += 1
        #adjacent_cars_start_df.to_csv(proj_cwd + "/output/adj_start_"+timestamp+".csv")
        num_vehs_adj_start = veh_count
        start_point_reached = True

start_time = driver.getTime()

#################################
CHNAGE_LANE_OFFSET = 0
CHANGE_LANE_START_TIME = CHNAGE_LANE_OFFSET
CHANGE_LANE_STOP_TIME = 2.5 + CHNAGE_LANE_OFFSET

CHANGE_LANE_BEHAVIOR = 0.08

##################################
driver.setCruisingSpeed(60)  #change to initial speed


current_state = LANE_KEEP

#print ('-----------------------------')
#print ('STATE : ' + CUR_STATE[current_state] + ", Time:" + str(driver.getTime()))
#print ('-----------------------------')
##################################



merge_count = 0

##################################




mpc = MPC()



def get_distance_from_merge_end_point(x,y):
    merge_last_point = (-91.26,63.83)
    return np.sqrt((x-merge_last_point[0])**2+ (y-merge_last_point[1])**2)

##################################
#          main
##################################

prev_time = driver.getTime()
prev_state = get_current_state(gps,compass,driver,current_state)
a = 0
initial_angle = 1
##################################
# Collect variables at each time stamp
while driver.step() != -1 and t < TIME_DEF + 18:



  t = driver.getTime()

  gps_val = gps.getValues()

  x , y , psi , v ,b = get_current_state(gps,compass,driver,current_state)
  v_state = [x,y,psi,v,b]
  dt = t-prev_time
  #print ('dt = ', dt)
  old_speed_merge = speed_merge
  speed_merge = gps.getSpeed()

  old_gap_lead = gap_lead
  old_speed_lead = speed_lead

  old_gap_lag = gap_lag
  old_speed_lag = speed_lag

  gap_lead, speed_lead = get_front_vehicle_distance(radar_f, v_state)
  gap_lag, speed_lag = get_back_vehicle_distance(radar_b, v_state)

  if(dt>0):
      acc = (v_state[3]- prev_state[3])/(dt)
      v_state.append(acc)
      old_acce_merge = acce_merge
      acce_merge = (speed_merge - old_speed_merge) / dt

      if (speed_lead == DEFAULT_OUT_SPEED or old_speed_lead == DEFAULT_OUT_SPEED):
          acce_lead = 0
      else:
          acce_lead = (speed_lead - old_speed_lead) / dt
      if (speed_lag == DEFAULT_OUT_SPEED or old_speed_lag == DEFAULT_OUT_SPEED):
          acce_lag = 0
      else:
          acce_lag = (speed_lag - old_speed_lag) / dt


  if(current_state == LANE_KEEP):
      dist_left = get_distance_from_merge_end_point(x,y)
      remaining_dist = dist_left
      ##print ('Distance Left: (m) ', dist_left)
      prev_a = a
      a = idm(v, dist_left, v)
      acce_change = acce_merge - old_acce_merge
      #print('a ' + str(a))
      driver.setCruisingSpeed(a*3.6)
      #speed_merge = a*3.6
      #acce_merge = a

      if(dist_left <= 100 and merge_point_reached is False):
          veh_count = 1
          for target in radar_f.getTargets():
              #adjacent_cars_merge_df.loc[veh_count] = [target.distance, target.received_power, target.speed, target.azimuth]
              veh_count += 1
          for target in radar_l.getTargets():
              #adjacent_cars_merge_df.loc[veh_count] = [target.distance, target.received_power, target.speed, target.azimuth]
              veh_count += 1
          for target in radar_b.getTargets():
              #adjacent_cars_merge_df.loc[veh_count] = [target.distance, target.received_power, target.speed, target.azimuth]
              veh_count += 1
          #adjacent_cars_merge_df.to_csv(proj_cwd + "/output/adj_merge_"+timestamp+".csv")
          num_vehs_adj_merge = veh_count
          merge_point_reached = True

      if(dist_left<100):
          # check the sidewise vehicles
          # if enough gap then change the state to PERFORM_MERGING

          # this is gap based model.
          #do_change_lane = change_lane_on_gap(radar_b,radar_f,radar_l)

          # this is risk-based model
          do_change_lane = change_lane_on_risk(radar_b,radar_f,radar_l,v_state,dist_left, acce_lead,acce_lag)


          lane_change_q.append(do_change_lane)

          if(len(lane_change_q)==3):
              if(target_speed == 0):
                  target_speed = driver.getTargetCruisingSpeed()

              if(lane_change_q[0] and lane_change_q[1] and lane_change_q[2]):
                  current_state = MERGINING
                  driver.setCruisingSpeed(target_speed)

              elif(dist_left < 60 and
              (target_speed - driver.getTargetCruisingSpeed()) < 3):
                  driver.setCruisingSpeed(driver.getTargetCruisingSpeed()*.98)

          #print ('change change status : ', do_change_lane)

  else:
      collision_check()
      ############### MPC #####################
      cur_veh_state = get_current_state(gps,compass,driver,current_state)
      cur_veh_state.append(acce_merge)
      cte = mpc.cte(1,cur_veh_state[0],cur_veh_state[1])
      ##print (cur_veh_state , 'cte : ', mpc.cte(1,cur_veh_state[0],cur_veh_state[1]) , ' m ')

      ##print (cur_veh_state, '\t\t with cte : ',cte)
      spd = 0
      dist = 0
      #a = 0
      #print("cte " + str(cte))
      if(cte<0.1):
          current_state = PERFORMED_MERGING
          driver.setIndicator(Driver.INDICATOR_OFF)
      #if(current_state==MERGINING):
      v_state = mpc.run(cur_veh_state)
      ##print("v_state:")
      ##print(v_state[1])
      driver.setSteeringAngle(v_state[1]*initial_angle)
      steering_merge = v_state[1] * initial_angle
      #initial_angle *= .99
      ##print(initial_angle)
      ##print(steering_merge)


        #if(risk_data_df.at[t-start_time-.01,'acce_change'] > 3):
            #if(risk_data_df.at[t-start_time-.02,'acce_change'] > 3):
        #print("ERROR: JERK THRESHOLD EXCEEDED (Collision Detected)")
        #risk_data_df.to_csv(proj_cwd + r"/output/data_"+timestamp+"_"+random_file_flag+".csv")
        #driver.simulationQuit(200)
        #del driver



      #########################################
      if(current_state == MERGINING):
          driver.setIndicator(Driver.INDICATOR_LEFT)
          spd = driver.getTargetCruisingSpeed() + v_state[0]*0.04
          acce_change = acce_merge - old_acce_merge
          ##print("v_state[0]:")
          ##print(v_state[0])
          #print(spd)
          #speed_merge = spd
          driver.setCruisingSpeed(spd)



      else:
          # use idm model.
          # get fron vehicle distance and speed
          if(abs(cur_veh_state[2]-1.25)<0.1):
              # this means that the vehicle is aligned in the right direction.
              # get the front vehicle distance.
              dist, spd = get_front_vehicle_distance(radar_f,v_state)
              lead_veh_dist_tracking.append(dist)
              lead_veh_spd_tracking.append(spd)

              #speed_lead = spd
              #gap_lead = dist


              dist = np.array(lead_veh_dist_tracking).min()
              spd  = np.array(lead_veh_spd_tracking).min()

              ##print ('after merging : ', dist , ' spd : ', spd)
              if(dist!=10000):
                  prev_a = a
                  a = idm(cur_veh_state[3], dist, -spd)
                  acce_change = a - prev_a
                  #print('a ' + str(a))
                  #print ('recommandded speed : ', a*3.6, ' kmh')
                  driver.setCruisingSpeed(a*3.6)
                  #acce_merge = a
                  #speed_merge = a*3.6
              else :
                  driver.setCruisingSpeed(DEFAULT_OUT_SPEED)
                  a = 0
                  #speed_merge = NORMAL_SPD



          #pass


      #spd =  + v_state[0]*0.5
  cur_veh_state = get_current_state(gps,compass,driver,current_state)
  cur_veh_state.insert(0,driver.getTime())
  cur_veh_state.append(current_state)




  try:
      TTCf = gap_lead/(speed_merge - speed_lead)
      if TTCf < 0:
          TTCf = DEFAULT_OUT_TTC
  except:
      TTCf = DEFAULT_OUT_TTC

  try:
      TTCb = gap_lag/(speed_lag - speed_merge)
      if TTCb < 0:
          TTCb = DEFAULT_OUT_TTC
  except:
      TTCb = DEFAULT_OUT_TTC

  if (TTCf > 2 and TTCb > 2) or current_state == PERFORMED_MERGING or current_state == LANE_KEEP:
      risk_status = 0
  else:
      risk_status = 1
  ##print ("AV_spd",v_state[3])

  risk_data_df.loc[(t - start_time)] = [speed_merge,acce_merge,steering_merge,speed_lead,acce_lead,gap_lead,
  speed_lag,acce_lag,gap_lag,remaining_dist,num_vehs_adj_start,num_vehs_adj_merge,
  TTCf, TTCb, risk_status, acce_change]

  prev_time  = t
  prev_state = v_state



# Output to csv file - trajectory and risk
#trajectory_profile_df.to_csv(proj_cwd + r"/output/tp_"+timestamp+".csv")
##print("tp written to: " + proj_cwd + r"/output/tp_"+timestamp+".csv")
#risk_profile_df.to_csv(proj_cwd + r"/output/rp_"+timestamp+".csv")
##print("rp written to: " + proj_cwd + r"/output/tp_"+timestamp+".csv")
risk_data_df.to_csv(proj_cwd + r"/output/data_"+timestamp+"_"+random_file_flag+
"_"+str(COLLISION_FLAG)+".csv")
driver.simulationQuit(200)

del driver
