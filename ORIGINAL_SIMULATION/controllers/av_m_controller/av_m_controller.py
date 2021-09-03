import math
import time
import json
import random
from vehicle import Driver
from controller import Camera, GPS
from controller import Receiver, Lidar, Radar,RadarTarget, Compass
import cv2
import numpy as np
from LaneKeeping import *
from haversine import haversine, Unit
from shapely.geometry import Point, Polygon
from mpc import *
from state import *
from lane_change import *
from lane_change_risk import *
import os, os.path

from collections import deque
lane_change_q = deque(maxlen=3)
lead_veh_dist_tracking = deque(maxlen=4)
lead_veh_spd_tracking = deque(maxlen=4)

driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(0.0)

receiver = Receiver("receiver")
receiver.enable(10)

camera_l = Camera("camera_left")
camera_l.enable(10)

camera_r = Camera("camera_right")
camera_r.enable(10)

radar_b = Radar("esr_back")
radar_b.enable(10)

radar_l = Radar("esr_left")
radar_l.enable(10)

radar_f = Radar("esr_front")
radar_f.enable(10)

compass = Compass("compass")
compass.enable(10)
#camera.recognitionEnable(50)
count = 0

gps = GPS("gps")
gps.enable(10)


#lidar = Lidar("lms291")
#lidar.enable(10)
#lidar.enablePointCloud()
# save trajectory data from SUMO

proj_cwd = os.getcwd()
proj_cwd = os.path.abspath(__file__+'/../../../..')

output_SUMO_Webots_rajectory_profile = open(proj_cwd+"\\output_SUMO_Webots_rajectory_profile.txt", "w")
def trajectory_profile(cur_veh_state):

    out_path = proj_cwd+""
    file_path = out_path + "\\output_SUMO_trajectory_profile.txt"
    file = open(file_path,mode='r')
    all_of_it = file.read()

    trajectory_profile = all_of_it.split(' ')

    print (cur_veh_state,trajectory_profile,file=output_SUMO_Webots_rajectory_profile)
    file.close()

output_crash_risk_profile = open(proj_cwd+"\\crash_risk_profile.txt", "w")
def risk_profile(cur_veh_state):
    out_path = proj_cwd+""
    file_path = out_path + "\\output_crash_risk.txt"
    file = open(file_path,mode='r')
    all_of_it = file.read()

    risk_profile = all_of_it.split(' ')

    print (cur_veh_state,risk_profile,file=output_crash_risk_profile)
    file.close()

file_state = "vehicle_state.txt"

with open(file_state,'w') as f:
    f.close()


def save_state(time, state, speed):
    with open("vehicle_state.txt", 'a') as f:
        f.write(str(time) +','+ str(state)+','+str(speed)+'\n')
        f.close()

def sigmoid(x):
    return (1/(1+math.exp(-x)))

def del_sig(x):
    return sigmoid(x)*(1-sigmoid(x))

##########################################################
LANE_KEEP = 0
MERGINING = 1
PERFORMED_MERGING = 2
current_state = LANE_KEEP
CUR_STATE = ["MERGINING","LANE_KEEP", "MERGING"]
NORMAL_SPD = 104
########################################################

TIME_DEF = 30

########################################################

is_front = False
is_back = False
stop_lane_change = False
start_lane_change = False

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
    v_max = 29 #m/s

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

def is_blind_spot_vehicle():
  global radar_b,GAP_LAG

  # process the back radar
  n_targets =  radar_b.getNumberOfTargets()
  targets = radar_b.getTargets()

  #print ('total targets: ', n_targets)
  blind_spot = False
  #print ("Back Radar")
  for i in range(n_targets):
      dist =  targets[i].distance
      speed = targets[i].speed
      azi = targets[i].azimuth
      K = dist*azi

      #print ('Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6+driver.getCurrentSpeed()), ' theta:', targets[i].azimuth )

      if(K>2 and K<5 and dist>2 and dist<GAP_LAG):
          print ("back vehicle on blind spot")

      #if((dist>5 and dist<15)):
          #start_lane_change = False
      #    print ("vehicle in blind spot")
          blind_spot=True
  # process the back radar
  n_targets =  radar_l.getNumberOfTargets()
  targets = radar_l.getTargets()
  #print ("Side Radar")
  for i in range(n_targets):
      dist =  targets[i].distance
      speed = targets[i].speed
      azi = targets[i].azimuth
      K = dist*azi

      if(dist>3.0 and dist<7):
          #print ('side vehicle in blind spot')
          blind_spot=True

      #print ('Side Radar: \t Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',targets[i].speed, ' theta:', targets[i].azimuth )

  return blind_spot
#############################################################

risk_lane_change = False

def risk_based_merge():
    global risk_lane_change
    out_path = proj_cwd+"\\acc.txt"
    file_path = out_path + "\\output_merge_command.txt"
    file = open(file_path,mode='r')
    all_of_it = file.read()

    if(all_of_it.strip()=="0"):risk_lane_change = False
    else: risk_lane_change = True


    #if (risk_lane_change==False):
    #    print ("<<<<<<< Not safe to merge >>>>")
    #else: print ("<<<<<<< Safe to merge >>>>")


    #print ('Merge Command : ',all_of_it )
    file.close()



def change_lane(front_dist = 0 , back_dist = 0):
    global stop_lane_change,is_front, is_back,start_lane_change, driver

    #print ('Lead vehicle dist: ', front_dist, ' Follower vehicle dist: ', back_dist)
    global GAP_LEAD
    global GAP_LAG

    if(is_front and is_back):
        if(front_dist>GAP_LEAD and back_dist>GAP_LAG):
            start_lane_change= True
        else:
            start_lane_change =False

    elif(is_front==False and is_back==True):
        if(back_dist>GAP_LAG):
            start_lane_change = True
        else:
            start_lane_change = False

    elif(is_front==True and is_back==False):
        if(front_dist>GAP_LEAD):
            start_lane_change = True
        else :
            start_lane_change = False

    else:
        start_lane_change = True

    if(start_lane_change==True):
        #print ("---------------------------------")
        print ("<<<< Clear to changle Lane >>>>>")
        #print ("---------------------------------")

leader_spd = -1
leader_dist = 1000.0

def decode_rx_msg(message):

    global stop_lane_change,is_front, is_back,vehicle_speeds,risk_lane_change,file_time
    global current_state,LANE_KEEP,PERFORM_MERGING,leader_spd
    global leader_dist

    for line in message.splitlines():
        loaded_json = json.loads(line)
        #print (loaded_json)

        if(loaded_json['eid']==1):
            if(loaded_json['vid']=='webotsVehicle0'):
                stop_lane_change = True

                print ('<<<<<<<< Merging Done !! >>>>>>>>')


                if(current_state == PERFORM_MERGING):
                    stop_lane_change= True
                    risk_lane_change = False
                    current_state = LANE_KEEP
                    print ("<<<<<<<<<<<< Merging Done >>>>>>>>>>>")

                if (current_state == LANE_KEEP):
                    leader_spd = float(loaded_json['l_spd'])
                    leader_dist = float(loaded_json['l_dist'])

            else:
                vehicle_speeds[loaded_json['vid']] = float(loaded_json['spd'])

        if(loaded_json['eid']==3):
            print ('webots vehicle info:')

            x = float(loaded_json['x'])
            y = float(loaded_json['y'])

            x1 , y1 = 878.45 , 413.03
            x2 , y2 = 968.32, 441.91
            x3 , y3 = 964.18, 444.74
            x4 , y4 = 875.22,414.92
            ### if the vehicle is inside a rectangle

            coords = [(x1,y1), (x2,y2),
                      (x3,y3), (x4,y4)]

            poly = Polygon(coords)

            p = Point(x,y)

            b = poly.contains(p)

            if(b):
                stop_lane_change = True

                if(current_state == PERFORM_MERGING):
                    stop_lane_change= True
                    risk_lane_change = False
                    current_state = LANE_KEEP
                    print ("<<<<<<<<<<<< Merging Done >>>>>>>>>>>")

                print ('inside main lane')
            else:
                print ('outside')

        ###############################
        #   Lnae chaning parameter extraction
        ###############################

        """
        elif(loaded_json['eid']==2):

            front_dist = 0
            back_dist = 0

            if("left_leader_dist" in loaded_json):
                #print ('Leader vehicle distance : ', loaded_json["left_leader_dist"])
                is_front = True
                front_dist = float(loaded_json["left_leader_dist"])
            else :
                is_front = False

            if("left_follower_dist" in loaded_json):
                #print ('Follower vehicle distance : ', loaded_json["left_follower_dist"])
                is_back = True
                back_dist = float(loaded_json["left_follower_dist"])
            else :
                is_back = False

            # in risk based modeing it will not check again is there is
            # enough gap or not.

            #change_lane(front_dist = front_dist,back_dist = back_dist)
        """
        #print (loaded_json)

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

def control_module_init():
    global driver, last_time, err_sum, last_error

    last_time = driver.getTime()

    if np.isnan(err_sum):
        err_sum = 0

    last_error = 0.0

def distance(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2+ (p1[1]-p2[1])**2)

def control_module(value):

    global driver, last_time, err_sum, last_error, target, kp, kd, ki

    t = driver.getTime()

    del_time = t - last_time

    if np.isnan(err_sum):
        err_sum = 0

    if(del_time==0): return 0

    error = value-target
    err_sum = err_sum + error*del_time

    #print (error*del_time)

    d_error = (error-last_error)/del_time



    output = kp*error + ki*err_sum + kd*d_error

    #print ('error: ', error, ' del_time: ', del_time , ' d_error : ', d_error , ' error_sum:', err_sum, ' output :', output, )

    last_time = t
    last_error = error


    if(output>0.25): output = 0.25
    if(output<-0.25):output = -0.25

    return output



prev_t = driver.getTime()

##################################
#      Delay Routine
##################################
while(driver.step() != -1):
    t = driver.getTime()


    if(t- prev_t>TIME_DEF):
        break

#################################
CHNAGE_LANE_OFFSET = 0
CHANGE_LANE_START_TIME = CHNAGE_LANE_OFFSET
CHANGE_LANE_STOP_TIME = 2.5 + CHNAGE_LANE_OFFSET

CHANGE_LANE_BEHAVIOR = 0.08

##################################
driver.setCruisingSpeed(60)  #change to initial speed


current_state = LANE_KEEP

print ('-----------------------------')
print ('STATE : ' + CUR_STATE[current_state] + ", Time:" + str(driver.getTime()))
print ('-----------------------------')
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


##################################
while driver.step() != -1:

  t = driver.getTime()

  gps_val = gps.getValues()

  x , y , psi , v ,b = get_current_state(gps,compass,driver,current_state)
  v_state = [x,y,psi,v,b]
  dt = t-prev_time
  print ('dt = ', dt)

  if(dt>0):
      acc = (v_state[3]- prev_state[3])/(dt)
      v_state.append(acc)


  if(current_state == LANE_KEEP):
      dist_left = get_distance_from_merge_end_point(x,y)
      #print ('Distance Left: (m) ', dist_left)
      a = idm(v, dist_left, v)
      driver.setCruisingSpeed(a*3.6)

      if(dist_left<100):
          # check the sidewise vehicles
          # if enough gap then change the state to PERFORM_MERGING

          # this is gap based model.
          #do_change_lane = change_lane_on_gap(radar_b,radar_f,radar_l)

          # this is risk-based model
          do_change_lane = change_lane_on_risk(radar_b,radar_f,radar_l,v_state,dist_left)


          lane_change_q.append(do_change_lane)

          if(len(lane_change_q)==3):
              if(lane_change_q[0] and lane_change_q[1] and lane_change_q[2]):
                  current_state = MERGINING

          print ('change change status : ', do_change_lane)

  else:
      ############### MPC #####################
      cur_veh_state = get_current_state(gps,compass,driver,current_state)
      cte = mpc.cte(1,cur_veh_state[0],cur_veh_state[1])
      #print (cur_veh_state , 'cte : ', mpc.cte(1,cur_veh_state[0],cur_veh_state[1]) , ' m ')

      print (cur_veh_state, '\t\t with cte : ',cte)


      if(cte<0.8):
          current_state = PERFORMED_MERGING
          driver.setIndicator(Driver.INDICATOR_OFF)
      #if(current_state==MERGINING):
      v_state = mpc.run(cur_veh_state)
      driver.setSteeringAngle(v_state[1]*0.25)

      #########################################
      if(current_state == MERGINING):
          driver.setIndicator(Driver.INDICATOR_LEFT)
          spd = driver.getCurrentSpeed() + v_state[0]*0.04
          driver.setCruisingSpeed(spd)
      else:
          # use idm model.
          # get fron vehicle distance and speed
          if(abs(cur_veh_state[2]-1.25)<0.1):
              # this means that the vehicle is aligned in the right direction.
              # get the front vehicle distance.
              dist, spd = get_front_vehicle_distance(radar_f)
              lead_veh_dist_tracking.append(dist)
              lead_veh_spd_tracking.append(spd)

              dist = np.array(lead_veh_dist_tracking).min()
              spd  = np.array(lead_veh_spd_tracking).min()

              print ('after merging : ', dist , ' spd : ', spd)
              if(dist!=10000):
                  a = idm(cur_veh_state[3], dist, -spd)
                  print ('recommandded speed : ', a*3.6, ' kmh')
                  driver.setCruisingSpeed(a*3.6)
              else :
                  driver.setCruisingSpeed(NORMAL_SPD)
          #pass

      #spd =  + v_state[0]*0.5
  cur_veh_state = get_current_state(gps,compass,driver,current_state)
  cur_veh_state.insert(0,driver.getTime())
  cur_veh_state.append(current_state)
  trajectory_profile(cur_veh_state)
  risk_profile(cur_veh_state)

  print ("AV_spd",v_state[3])


  prev_time  = t
  prev_state = v_state
