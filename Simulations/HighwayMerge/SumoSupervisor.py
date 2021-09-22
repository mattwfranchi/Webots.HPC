# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SumoSupervisor class inheriting from Supervisor."""

from controller import Supervisor, Node
from Objects import Vehicle, TrafficLight
from WebotsVehicle import WebotsVehicle


#from __future__ import absolute_import
#from __future__ import print_function

import os
import sys
import math
import json
import random
import numpy as np
import traceback

from scipy.stats import lognorm
from scipy.stats import uniform
from scipy.stats import weibull_min
from scipy.stats import gamma

hiddenPosition = 10000

################################
# Risk-based Predictive Model developed using NGSIM data
################################
#lag crash risk prediction
def risk_lag_prior(Speed_Merge,Acce_Merge,Remaining_Distance,Acce_lag,Gap_lag,Speed_dif):
    beta=[-0.648166,0.291651,-2.67226,0.010325,2.736206,-1.9484,3.314949] #beta=[intercept,variable1,variable2,...] 
    X=[1,Speed_Merge,Acce_Merge,Remaining_Distance,Acce_lag,Gap_lag,Speed_dif]
    Pr_lag_prior=1/(1+np.exp(-np.dot(beta,X)))
    #print("Pr_lag_prior",Pr_lag_prior)
    return Pr_lag_prior

def lag_conflict_likelihood(Gap_lag):
    #weibull cdf
    c=1.177935
    loc=0
    scale=15.060868 
    x_upper=Gap_lag+0.5
    x_lower=Gap_lag-0.5
    Pr_gap_lag_conflict=weibull_min.cdf(x_upper,c,loc,scale)-weibull_min.cdf(x_lower, c,loc,scale)
    return Pr_gap_lag_conflict
    
def lag_nonconflict_likelihood(Gap_lag):
 
    shape = 3.82145718
    rate = 0.06710455
    #gamma cdf
    loc=0
    scale=1/rate
    x_upper=Gap_lag+0.5
    x_lower=Gap_lag-0.5
    Pr_gap_lag_nonconflict=gamma.cdf(x_upper,shape,loc,scale)-gamma.cdf(x_lower,shape,loc,scale)
    return Pr_gap_lag_nonconflict



def risk_lag_posterior(Gap_lag,Pr_lag_prior,Pr_gap_lag_nonconflict,Pr_gap_lag_conflict):

    #print ('1-Pr_lag_prior  = ', 1-Pr_lag_prior)
    #print ('Pr_gap_lag_conflict * Pr_lag_prior = ', Pr_gap_lag_conflict * Pr_lag_prior)
    #print ('((Pr_gap_lag_conflict * Pr_lag_prior) + Pr_gap_lag_nonconflict *(1-Pr_lag_prior)) = ', ((Pr_gap_lag_conflict * Pr_lag_prior) + Pr_gap_lag_nonconflict *(1-Pr_lag_prior)))
    #print ('Pr_lag_prior * Pr_gap_lag_conflict = ', Pr_lag_prior * Pr_gap_lag_conflict)

    denom = ((Pr_gap_lag_conflict * Pr_lag_prior) + Pr_gap_lag_nonconflict *(1-Pr_lag_prior))

    if(denom == 0):
        return 1.0
    
    Pr_lag_posterior= (Pr_lag_prior * Pr_gap_lag_conflict)/denom
    #print ('Pr_lag_posterior: ', Pr_lag_posterior)

    #print ('')
    return Pr_lag_posterior


def risk_lead_prior(Acce_Merge,Remaining_Distance,Speed_lead,Acce_lead,Gap_lead):
    beta=[-0.871,-1.257,0.029,-0.034,0.451,-0.301]
    X=[1,Acce_Merge,Remaining_Distance,Speed_lead,Acce_lead,Gap_lead+2.5]
    Pr_lead_prior=1/(1+np.exp(-np.dot(beta,X)))
    return Pr_lead_prior

def lead_conflict_likelihood(Gap_lead):
    #uniform cdf
    loc= -4.996666
    scale=52.099515+4.996666
    x_upper=Gap_lead+0.5
    x_lower=Gap_lead-0.5
    Pr_gap_lead_conflict=uniform.cdf(x_upper,loc,scale)-uniform.cdf(x_lower,loc,scale)
    return Pr_gap_lead_conflict
    
def lead_nonconflict_likelihood(Gap_lead):
    #weibull cdf
    c= 1.609829 
    loc=0
    scale=49.765264
    x_upper=Gap_lead+0.5
    x_lower=Gap_lead-0.5
    Pr_gap_lead_conflict=weibull_min.cdf(x_upper,c,loc,scale)-weibull_min.cdf(x_lower, c,loc,scale)
    return Pr_gap_lead_conflict



def risk_lead_posterior(Gap_lead,Pr_lead_prior,Pr_gap_lead_nonconflict,Pr_gap_lead_conflict):
    denom = (Pr_gap_lead_conflict*Pr_lead_prior+Pr_gap_lead_nonconflict*(1-Pr_lead_prior))
    if(denom == 0):
        return 1.0    
    Pr_lead_posterior=Pr_lead_prior*Pr_gap_lead_conflict/(Pr_gap_lead_conflict*Pr_lead_prior+Pr_gap_lead_nonconflict*(1-Pr_lead_prior))
    return Pr_lead_posterior



def safety_distance_min(Speed_Merge,Speed_lead):
    b_av=4.6
    b_lead=4.2
    tau_av=0.9
    S_min=Speed_Merge*tau_av+1/2*np.square(Speed_Merge)/b_av-1/2*np.square(Speed_lead)/b_lead
    return S_min

 #compute TIT, referred to as the entity of the TTC lower than the TTC threshold. 
 
def TIT(traci,step,segment_vehicleIDs):
    TIT_step=0
    conflict_indicator=0
    for vehicleID in segment_vehicleIDs:
        front_vehicle=traci.vehicle.getLeader(vehicleID)
        if front_vehicle is not None:
            front_vehicleID=front_vehicle[0]
            if front_vehicleID in segment_vehicleIDs:
                speed_subject = traci.vehicle.getSpeed(vehicleID)
                acce_subject = traci.vehicle.getAcceleration(vehicleID)
                speed_front = traci.vehicle.getSpeed(front_vehicleID)
                acce_front = traci.vehicle.getAcceleration(front_vehicleID)            
                delta_acce=acce_subject-acce_front
                delta_speed = speed_subject-speed_front
                coefs = [1/2*delta_acce,delta_speed,-front_vehicle[1]]
                r= np.roots(coefs)
                r= r[np.isreal(r)]
                if len(r)>=2:
                    if r[0] <0 and r[1]<0:
                        TTC_lead=float("inf")
                    elif r[0]<0 or r[1]<0:
                        TTC_lead=np.max(r)
                    elif r[0]>0 and r[1]>0:
                        TTC_lead=np.min(r)
                else:
                    TTC_lead=float("inf")
                if TTC_lead<=2 and TTC_lead>0:
                    conflict_indicator=(1/TTC_lead-1/2)
                else:
                    conflict_indicator=0
        TIT_step=TIT_step+conflict_indicator*0.2
    return TIT_step
 
 #compute TET
def TET(traci,step,segment_vehicleIDs,output_TTC_Evaluate):
    TET_step=0
    conflict_indicator=0
    for vehicleID in segment_vehicleIDs:
        front_vehicle=traci.vehicle.getLeader(vehicleID)
        if front_vehicle is not None:
            front_vehicleID=front_vehicle[0]
            if front_vehicleID in segment_vehicleIDs:
                speed_subject = traci.vehicle.getSpeed(vehicleID)
                acce_subject = traci.vehicle.getAcceleration(vehicleID)
                speed_front = traci.vehicle.getSpeed(front_vehicleID)
                acce_front = traci.vehicle.getAcceleration(front_vehicleID)            
                delta_acce=acce_subject-acce_front
                delta_speed = speed_subject-speed_front
                coefs = [1/2*delta_acce,delta_speed,-front_vehicle[1]]
                r= np.roots(coefs)
                r= r[np.isreal(r)]
                if len(r)>=2:
                    if r[0] <0 and r[1]<0:
                        TTC_lead=float("inf")
                    elif r[0]<0 or r[1]<0:
                        TTC_lead=np.max(r)
                    elif r[0]>0 and r[1]>0:
                        TTC_lead=np.min(r)
                    print(step,vehicleID,TTC_lead,file=output_TTC_Evaluate)
                else:
                    TTC_lead=float("inf")
                    print(step,vehicleID,TTC_lead,file=output_TTC_Evaluate)
                if TTC_lead<=2:
                    conflict_indicator=1
                else:
                    conflict_indicator=0
        TET_step=TET_step+conflict_indicator*0.2
    return TET_step

def TTC_merge(traci,step,av_ID,output_lead_veh,output_lag_veh,MergingVehicleID):
    Speed_Merge = traci.vehicle.getSpeed(av_ID)
    Acce_Merge = traci.vehicle.getAcceleration(av_ID)
    Remaining_Distance = 99.49-traci.vehicle.getLanePosition(av_ID)
    leftLeaders = traci.vehicle.getLeftLeaders(av_ID)
    leftFollowers = traci.vehicle.getLeftFollowers(av_ID)
                        
    if len(leftLeaders)>=1:
        leftLeadersVehID = leftLeaders[0][0]
        Speed_lead = traci.vehicle.getSpeed(leftLeadersVehID)
        Acce_lead = traci.vehicle.getAcceleration(leftLeadersVehID)
        Gap_lead=leftLeaders[0][1]
        is_lead=True
        delta_acce=Acce_Merge-Acce_lead
        delta_speed = Speed_Merge-Speed_lead
        coefs = [1/2*delta_acce,delta_speed,-leftLeaders[0][1]]
        r= np.roots(coefs)
        r= r[np.isreal(r)]
        if len(r)>=2:
            if r[0] <0 and r[1]<0:
                TTC_lead=float("inf")
            elif r[0]<0 or r[1]<0:
                TTC_lead=np.max(r)
            elif r[0]>0 and r[1]>0:
                TTC_lead=np.min(r)
            if av_ID in MergingVehicleID:
                print(step,av_ID,TTC_lead,is_lead,file=output_lead_veh)
        else:
            TTC_lead=float("inf")
            if av_ID in MergingVehicleID:
                print(step,av_ID,TTC_lead,is_lead,file=output_lead_veh)
    else:
        is_lead =False
        TTC_lead=float("inf")
        print(step,av_ID,TTC_lead,is_lead,file=output_lead_veh)
    if len(leftFollowers)>=1:
        leftFollowersVehID = leftFollowers[0][0]
        Speed_lag =traci.vehicle.getSpeed(leftFollowersVehID)
        Acce_lag =traci.vehicle.getAcceleration(leftFollowersVehID)
        Gap_lag=leftFollowers[0][1]
        is_lag=True
        delta_acce=Acce_lag-Acce_Merge
        delta_speed = Speed_lag-Speed_Merge
        coefs = [1/2*delta_acce,delta_speed,-leftFollowers[0][1]]
        r= np.roots(coefs)
        r= r[np.isreal(r)]
        
        if len(r)>=2:
            if r[0] <0 and r[1]<0:
                TTC_lag=float("inf")
            elif r[0]<0 or r[1]<0:
                TTC_lag=np.max(r)
            elif r[0]>0 and r[1]>0:
                TTC_lag=np.min(r)
            if av_ID in MergingVehicleID:
                print(step,av_ID,TTC_lag,is_lag,file=output_lag_veh)
        else:
            TTC_lag=float("inf")
            print(step,av_ID,TTC_lag,is_lag,file=output_lag_veh)
    else:
        is_lag =False
        TTC_lag=float("inf")
        if av_ID in MergingVehicleID:
            print(step,av_ID,TTC_lag,is_lag,file=output_lag_veh)

################################

def rotation_from_yaw_pitch_roll(yaw, pitch, roll):
    """Compute the rotation from the roll pitch yaw angles."""
    rotation = [0, 1, 0, 0]
    # construct rotation matrix
    # a b c
    # d e f
    # g h i
    a = math.cos(roll) * math.cos(yaw)
    b = -math.sin(roll)
    c = math.cos(roll) * math.sin(yaw)
    d = math.sin(roll) * math.cos(yaw) * math.cos(pitch) + math.sin(yaw) * math.sin(pitch)
    e = math.cos(roll) * math.cos(pitch)
    f = math.sin(roll) * math.sin(yaw) * math.cos(pitch) - math.cos(yaw) * math.sin(pitch)
    g = math.sin(roll) * math.cos(yaw) * math.sin(pitch) - math.sin(yaw) * math.cos(pitch)
    h = math.cos(roll) * math.sin(pitch)
    i = math.sin(roll) * math.sin(yaw) * math.sin(pitch) + math.cos(yaw) * math.cos(pitch)
    # convert it to rotation vector
    cosAngle = 0.5 * (a + e + i - 1.0)
    if math.fabs(cosAngle) > 1:
        return rotation
    else:
        rotation[0] = f - h
        rotation[1] = g - c
        rotation[2] = b - d
        rotation[3] = math.acos(cosAngle)
        # normalize vector
        length = math.sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2])
        if length != 0:
            rotation[0] = rotation[0] / length
            rotation[1] = rotation[1] / length
            rotation[2] = rotation[2] / length
        if rotation[0] == 0 and rotation[1] == 0 and rotation[2] == 0:
            return [0, 1, 0, 0]
        else:
            return rotation


class SumoSupervisor (Supervisor):
    """This is the main class that implements the actual interface."""
    
    
    def get_viewpoint_position_field(self):
        """Look for the 'position' field of the Viewpoint node."""
        children = self.getRoot().getField("children")
        number = children.getCount()
        for i in range(0, number):
            node = children.getMFNode(i)
            if node.getType() == Node.VIEWPOINT:
                return node.getField("position")
        return None

    def get_initial_vehicles(self):
        """Get all the vehicles (both controlled by SUMO and Webots) already present in the world."""
        for i in range(0, self.vehiclesLimit):
            defName = "SUMO_VEHICLE%d" % self.vehicleNumber
            node = self.getFromDef(defName)
            if node:
                self.vehicles[i] = Vehicle(node)
                self.vehicles[i].name.setSFString("SUMO vehicle %i" % self.vehicleNumber)
                self.vehicleNumber += 1
            else:
                break
        for i in range(0, self.vehiclesLimit):
            defName = "WEBOTS_VEHICLE%d" % self.webotsVehicleNumber
            node = self.getFromDef(defName)
            if node:
                self.webotsVehicles[i] = WebotsVehicle(node, self.webotsVehicleNumber)
                self.webotsVehicleNumber += 1
            else:
                break

    def generate_new_vehicle(self, vehicleClass):
        """Generate and import a new vehicle that will be controlled by SUMO."""
        # load the new vehicle
        vehicleString, defName = Vehicle.generate_vehicle_string(self.vehicleNumber, vehicleClass)
        self.rootChildren.importMFNodeFromString(-1, vehicleString)
        self.vehicles[self.vehicleNumber] = Vehicle(self.getFromDef(defName))
        self.vehicleNumber += 1

    def get_vehicle_index(self, id, generateIfneeded=True):
        """Look for the vehicle index corresponding to this id (and optionnaly create it if required)."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].currentID == id:
                # the vehicle was already here at last step
                return i
        if not generateIfneeded:
            return -1
        # the vehicle was not present last step
        # check if a corresponding vehicle is already in the simulation
        node = self.getFromDef(id)
        if node and (node.getTypeName() in Vehicle.get_car_models_list() or
                     node.getTypeName() in Vehicle.get_bus_models_list() or
                     node.getTypeName() in Vehicle.get_truck_models_list() or
                     node.getTypeName() in Vehicle.get_motorcycle_models_list()):
            self.vehicles[self.vehicleNumber] = Vehicle(node)
            self.vehicles[self.vehicleNumber].currentID = id
            self.vehicleNumber += 1
            return self.vehicleNumber - 1
        # check if a vehicle is available
        vehicleClass = self.get_vehicle_class(id)
        for i in range(0, self.vehicleNumber):
            if not self.vehicles[i].inUse and self.vehicles[i].vehicleClass == vehicleClass:
                # if a vehicle is available assign it to this id
                self.vehicles[i].currentID = id
                self.vehicles[i].name.setSFString(id)
                return i
        # no vehicle available => generate a new one if limit is not reached
        if self.vehicleNumber < self.vehiclesLimit:
            vehicleClass = self.get_vehicle_class(id)
            self.generate_new_vehicle(vehicleClass)
            return self.vehicleNumber - 1
        return -1

    def get_vehicle_class(self, id):
        """Get the class of the vehicle associated to this id."""
        if id in self.vehiclesClass:
            return self.vehiclesClass[id]
        vehicleClass = Vehicle.get_corresponding_vehicle_class(self.traci.vehicle.getVehicleClass(id))
        self.vehiclesClass[id] = vehicleClass
        return vehicleClass

    def disable_unused_vehicles(self, IdList):
        """Check for all the vehicles currently used if they need to be disabled."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].inUse and self.vehicles[i].currentID not in IdList:
                self.vehicles[i].inUse = False
                self.vehicles[i].name.setSFString("SUMO vehicle %i" % i)
                self.vehicles[i].currentLane = None
                self.vehicles[i].currentRoad = None
                self.vehicles[i].laneChangeStartTime = None
                self.vehicles[i].laneChangeDistance = 0

    def hide_unused_vehicles(self):
        """Hide all the newly unused vehicles."""
        for i in range(0, self.vehicleNumber):
            if not self.vehicles[i].inUse:
                if self.vehicles[i].targetPos[0] != hiddenPosition:
                    self.vehicles[i].targetPos = [hiddenPosition, 0.5, i * 10]
                    self.vehicles[i].currentPos = [hiddenPosition, 0.5, i * 10]
                    self.vehicles[i].currentRot = [0, 1, 0, 0]
                    self.vehicles[i].targetRot = [0, 1, 0, 0]
                    self.vehicles[i].currentAngles = [0, 0, 0]
                    self.vehicles[i].targetAngles = [0, 0, 0]
                    self.vehicles[i].translation.setSFVec3f([hiddenPosition, 0.5, i * 10])
                    self.vehicles[i].node.setVelocity([0, 0, 0, 0, 0, 0])
                    for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                        wheelAngularVelocity.setSFVec3f([0, 0, 0])

    def stop_all_vehicles(self):
        """Stop all the vehicles (to be called when controller exits)."""
        for i in range(0, self.vehicleNumber):
            self.vehicles[i].node.setVelocity([0, 0, 0, 0, 0, 0])
            for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                wheelAngularVelocity.setSFVec3f([0, 0, 0])

    def get_vehicles_position(self, id, subscriptionResult, step, xOffset, yOffset,
                              maximumLateralSpeed, maximumAngularSpeed, laneChangeDelay):
        """Compute the new desired position and orientation for all the vehicles controlled by SUMO."""
        if subscriptionResult is None:
            return
        height = 0.4
        roll = 0.0
        pitch = 0.0
        sumoPos = subscriptionResult[self.traci.constants.VAR_POSITION]
        sumoAngle = subscriptionResult[self.traci.constants.VAR_ANGLE]
        pos = [-sumoPos[0] + xOffset, height, sumoPos[1] - yOffset]
        angle = math.pi * sumoAngle / 180
        dx = -math.cos(angle)
        dy = -math.sin(angle)
        yaw = -math.atan2(dy, -dx)
        # correct position (origin of the car is not the same in Webots / sumo)
        vehicleLength = subscriptionResult[self.traci.constants.VAR_LENGTH]
        pos[0] += 0.5 * vehicleLength * math.sin(angle)
        pos[2] -= 0.5 * vehicleLength * math.cos(angle)
        # if needed check the vehicle is in the visibility radius
        if self.radius > 0:
            viewpointPosition = self.viewpointPosition.getSFVec3f()
            xDiff = viewpointPosition[0] - pos[0]
            yDiff = viewpointPosition[1]
            zDiff = viewpointPosition[2] - pos[2]
            distance = math.sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff)
            if distance > self.radius:
                index = self.get_vehicle_index(id, generateIfneeded=False)
                if index >= 0:
                    self.vehicles[index].inUse = False
                    self.vehicles[index].currentID = ""
                    self.vehicles[index].name.setSFString("SUMO vehicle %i" % index)
                return
        index = self.get_vehicle_index(id)
        if index >= 0:
            vehicle = self.vehicles[index]
            height = vehicle.wheelRadius
            if self.enableHeight:
                roadID = subscriptionResult[self.traci.constants.VAR_ROAD_ID]
                roadPos = subscriptionResult[self.traci.constants.VAR_LANEPOSITION]
                if roadID.startswith(":"):
                    # this is a lane change it does not contains edge information
                    # in that case, use previous height, roll and pitch
                    height = vehicle.currentPos[1]
                    roll = vehicle.roll
                    pitch = vehicle.pitch
                else:
                    tags = roadID.split('_')
                    del tags[0]  # remove the first one which is the 'id' of the road
                    for tag in tags:
                        if tag.startswith('height'):
                            height = height + float(tag.split('height', 1)[1])
                        elif tag.startswith('roll'):
                            roll = float(tag.split('roll', 1)[1])
                        elif tag.startswith('pitch'):
                            pitch = float(tag.split('pitch', 1)[1])
                    vehicle.pitch = pitch
                    vehicle.roll = roll
                    # ajust height according to the pitch
                    if pitch != 0:
                        height += (roadPos - 0.5 * vehicleLength) * math.sin(pitch)
                    # ajust height according to the roll and lateral position of the vehicle
                    if roll != 0.0:
                        laneIndex = subscriptionResult[self.traci.constants.VAR_LANE_INDEX]
                        laneID = subscriptionResult[self.traci.constants.VAR_LANE_ID]
                        laneWidth = self.traci.lane.getWidth(laneID)
                        edge = self.net.getEdge(roadID)
                        numberOfLane = edge.getLaneNumber()
                        # compute lateral distance from the center of the lane
                        distance = math.fabs((laneIndex - numberOfLane / 2) + 0.5) * laneWidth
                        if laneIndex >= (numberOfLane / 2):
                            height = height - distance * math.sin(roll)
                        else:
                            height = height + distance * math.sin(roll)
            pos[1] = height
            if vehicle.inUse:
                # TODO: once the lane change model of SUMO has been improved
                #       (sub-lane model currently in development phase) we will be able to remove this corrections

                # compute lateral (x) and longitudinal (z) displacement
                diffX = pos[0] - vehicle.targetPos[0]
                diffZ = pos[2] - vehicle.targetPos[2]
                x1 = math.cos(-angle) * diffX - math.sin(-angle) * diffZ
                z1 = math.sin(-angle) * diffX + math.cos(-angle) * diffZ
                # check for lane change
                if (vehicle.currentRoad is not None and
                        vehicle.currentRoad == subscriptionResult[self.traci.constants.VAR_ROAD_ID] and
                        vehicle.currentLane is not None and
                        vehicle.currentLane != subscriptionResult[self.traci.constants.VAR_LANE_INDEX]):
                    vehicle.laneChangeStartTime = self.getTime()
                    vehicle.laneChangeDistance = x1
                x2 = x1
                # artificially add an angle depending on the lateral speed
                artificialAngle = 0
                if z1 > 0.0001:  # don't add the angle if speed is very small as atan2(0.0, 0.0) is unstable
                    # the '0.15' factor was found empirically and should not depend on the simulation
                    artificialAngle = 0.15 * math.atan2(x1, z1)
                if (vehicle.laneChangeStartTime is not None and
                        vehicle.laneChangeStartTime > self.getTime() - laneChangeDelay and
                        abs(vehicle.laneChangeDistance) >= abs(x1)):  # lane change case
                    ratio = (self.getTime() - vehicle.laneChangeStartTime) / laneChangeDelay
                    ratio = (0.5 + 0.5 * math.sin((ratio - 0.5) * math.pi))
                    p = vehicle.laneChangeDistance * ratio
                    x2 = x1 - (vehicle.laneChangeDistance - p)
                    artificialAngle = math.atan2(x2, z1)
                # limit lateral speed
                threshold = 0.001 * step * maximumLateralSpeed
                x2 = min(max(x2, -threshold), threshold)
                x3 = math.cos(angle) * x2 - math.sin(angle) * z1
                z3 = math.sin(angle) * x2 + math.cos(angle) * z1
                pos = [x3 + vehicle.targetPos[0], pos[1], z3 + vehicle.targetPos[2]]
                diffYaw = yaw - vehicle.targetAngles[1] - artificialAngle
                # limit angular speed
                while diffYaw > math.pi:
                    diffYaw -= 2 * math.pi
                while diffYaw < -math.pi:
                    diffYaw += 2 * math.pi
                threshold = 0.001 * step * maximumAngularSpeed
                diffYaw = min(max(diffYaw, -threshold), threshold)
                yaw = diffYaw + vehicle.targetAngles[1]
                # tilt motorcycle depending on the angluar speed
                if vehicle.type in Vehicle.get_motorcycle_models_list():
                    threshold = 0.001 * step * maximumLateralSpeed
                    roll -= min(max(diffYaw / (0.001 * step), -0.2), 0.2)
            rot = rotation_from_yaw_pitch_roll(yaw, pitch, roll)
            if not vehicle.inUse:
                # this vehicle was previously not used, move it directly to the correct initial location
                vehicle.inUse = True
                vehicle.currentPos = pos
                vehicle.currentRot = rot
                vehicle.currentAngles = [roll, yaw, pitch]
            else:
                vehicle.currentPos = vehicle.targetPos
                vehicle.currentRot = vehicle.targetRot
                vehicle.currentAngles = vehicle.targetAngles
            # update target and wheels speed
            vehicle.targetPos = pos
            vehicle.targetRot = rot
            vehicle.targetAngles = [roll, yaw, pitch]
            if self.traci.constants.VAR_SPEED in subscriptionResult:
                vehicle.speed = subscriptionResult[self.traci.constants.VAR_SPEED]
            vehicle.currentRoad = subscriptionResult[self.traci.constants.VAR_ROAD_ID]
            vehicle.currentLane = subscriptionResult[self.traci.constants.VAR_LANE_INDEX]

    def update_vehicles_position_and_velocity(self, step, rotateWheels):
        """Update the actual position (using angular and linear velocities) of all the vehicles in Webots."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].inUse:
                self.vehicles[i].translation.setSFVec3f(self.vehicles[i].currentPos)
                self.vehicles[i].rotation.setSFRotation(self.vehicles[i].currentRot)
                velocity = []
                velocity.append(self.vehicles[i].targetPos[0] - self.vehicles[i].currentPos[0])
                velocity.append(self.vehicles[i].targetPos[1] - self.vehicles[i].currentPos[1])
                velocity.append(self.vehicles[i].targetPos[2] - self.vehicles[i].currentPos[2])
                for j in range(0, 3):
                    diffAngle = self.vehicles[i].currentAngles[j] - self.vehicles[i].targetAngles[j]
                    if diffAngle > math.pi:
                        diffAngle = diffAngle - 2 * math.pi
                    elif diffAngle < -math.pi:
                        diffAngle = diffAngle + 2 * math.pi
                    velocity.append(diffAngle)
                velocity[:] = [1000 * x / step for x in velocity]
                self.vehicles[i].node.setVelocity(velocity)
                if rotateWheels:
                    angularVelocity = [self.vehicles[i].speed / self.vehicles[i].wheelRadius, 0, 0]
                    for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                        wheelAngularVelocity.setSFVec3f(angularVelocity)

    def update_webots_vehicles(self, xOffset, yOffset):
        """Update the position of all the vehicles controlled by Webots in SUMO."""
        for i in range(0, self.webotsVehicleNumber):
            if self.webotsVehicles[i].is_on_road(xOffset, yOffset, self.maxWebotsVehicleDistanceToLane, self.net):
                self.webotsVehicles[i].update_position(self.getTime(), self.net, self.traci, self.sumolib, xOffset, yOffset)
            else:
                # the controlled vehicle is not on any road
                # => we remove it from sumo network
                if self.webotsVehicles[i].name in self.traci.vehicle.getIDList():
                    self.traci.vehicle.remove(self.webotsVehicles[i].name)

    def get_traffic_light(self, IDlist):
        """Get the state of all the traffic lights controlled by SUMO."""
        self.trafficLightNumber = len(IDlist)
        self.trafficLights = {}
        LEDNames = []
        for i in range(0, self.getNumberOfDevices()):
            device = self.getDeviceByIndex(i)
            if device.getNodeType() == Node.LED:
                LEDNames.append(device.getName())
        for i in range(0, self.trafficLightNumber):
            id = IDlist[i]
            self.trafficLights[id] = TrafficLight()
            self.trafficLights[id].lightNumber = len(self.traci.trafficlight.getRedYellowGreenState(id))
            for j in range(0, self.trafficLights[id].lightNumber):
                trafficLightNode = self.getFromDef("TLS_" + id + "_" + str(j))
                if trafficLightNode is not None:
                    self.trafficLights[id].trafficLightRecognitionColors[j] = trafficLightNode.getField("recognitionColors")
                ledName = id + "_" + str(j) + "_"
                if (ledName + "r") in LEDNames:
                    self.trafficLights[id].LED[3 * j + 0] = self.getLED(ledName + "r")
                else:
                    self.trafficLights[id].LED[3 * j + 0] = None
                if (ledName + "y") in LEDNames:
                    self.trafficLights[id].LED[3 * j + 1] = self.getLED(ledName + "y")
                else:
                    self.trafficLights[id].LED[3 * j + 1] = None
                if (ledName + "g") in LEDNames:
                    self.trafficLights[id].LED[3 * j + 2] = self.getLED(ledName + "g")
                else:
                    self.trafficLights[id].LED[3 * j + 2] = None

    def update_traffic_light_state(self, id, states):
        """Update the traffic lights state in Webots."""
        # update light LED state if traffic light state has changed
        currentState = states[self.traci.constants.TL_RED_YELLOW_GREEN_STATE]
        if self.trafficLights[id].previousState != currentState:
            self.trafficLights[id].previousState = currentState
            for j in range(0, self.trafficLights[id].lightNumber):
                # Update red LED if it exists
                if self.trafficLights[id].LED[3 * j + 0]:
                    if currentState[j] == 'r' or currentState[j] == 'R':
                        self.trafficLights[id].LED[3 * j + 0].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [1, 0, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 0].set(0)
                # Update yellow LED if it exists
                if self.trafficLights[id].LED[3 * j + 1]:
                    if currentState[j] == 'y' or currentState[j] == 'Y':
                        self.trafficLights[id].LED[3 * j + 1].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [1, 0.5, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 1].set(0)
                # Update green LED if it exists
                if self.trafficLights[id].LED[3 * j + 2]:
                    if currentState[j] == 'g' or currentState[j] == 'G':
                        self.trafficLights[id].LED[3 * j + 2].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [0, 1, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 2].set(0)

    def run(self, port, disableTrafficLight, directory, step, rotateWheels,
            maxVehicles, radius, enableHeight, useDisplay, displayRefreshRate,
            displayZoom, displayFitSize, maximumLateralSpeed, maximumAngularSpeed,
            laneChangeDelay, traci, sumolib):
        """Main loop function."""
        try:
            print('Connect to SUMO... This operation may take a few seconds.')
            self.step(step)
            traci.init(port, numRetries=20)
        except:
            sys.exit('Unable to connect to SUMO, please make sure any previous instance of SUMO is closed.\n You can try'
                     ' changing SUMO port using the "--port" argument.')

        self.traci = traci
        self.sumolib = sumolib
        self.radius = radius
        self.enableHeight = enableHeight
        self.sumoClosed = False
        self.temporaryDirectory = directory
        self.rootChildren = self.getRoot().getField("children")
        self.viewpointPosition = self.get_viewpoint_position_field()
        self.maxWebotsVehicleDistanceToLane = 5
        self.webotsVehicleNumber = 0
        self.webotsVehicles = {}
        self.vehicleNumber = 0
        self.vehicles = {}
        self.vehiclesLimit = maxVehicles
        self.vehiclesClass = {}
        self.count_steps = 0

    
        # for backward compatibility
        if self.traci.constants.TRACI_VERSION <= 15:
            self.traci.trafficlight = self.traci.trafficlights

        # get sumo vehicles already present in the world
        self.get_initial_vehicles()

        # parse the net and get the offsets
        self.net = sumolib.net.readNet((directory + "/sumo.net.xml").replace('/', os.sep))
        xOffset = self.net.getLocationOffset()[0]
        yOffset = self.net.getLocationOffset()[1]

        # Load plugin to the generic SUMO Supervisor (if any)
        self.usePlugin = False
        if os.path.exists((directory + "/plugin.py").replace('/', os.sep)):
            self.usePlugin = True
            sys.path.append(directory)
            import plugin
            sumoSupervisorPlugin = plugin.SumoSupervisorPlugin(self, self.traci, self.net)

        # Get all the LEDs of the traffic lights
        if not disableTrafficLight:
            trafficLightsList = self.traci.trafficlight.getIDList()
            self.get_traffic_light(trafficLightsList)
            for id in trafficLightsList:
                # subscribe to traffic lights state
                self.traci.trafficlight.subscribe(id, [self.traci.constants.TL_RED_YELLOW_GREEN_STATE])

        # Subscribe to new vehicles entering the simulation
        self.traci.simulation.subscribe([
            self.traci.constants.VAR_DEPARTED_VEHICLES_IDS,
            self.traci.constants.VAR_MIN_EXPECTED_VEHICLES
        ])

        # Create the vehicle variable subscription list
        self.vehicleVariableList = [
            self.traci.constants.VAR_POSITION,
            self.traci.constants.VAR_ANGLE,
            self.traci.constants.VAR_LENGTH,
            self.traci.constants.VAR_ROAD_ID,
            self.traci.constants.VAR_LANE_INDEX
        ]
        if rotateWheels:
            self.vehicleVariableList.append(self.traci.constants.VAR_SPEED)
        if enableHeight:
            self.vehicleVariableList.extend([
                self.traci.constants.VAR_ROAD_ID,
                self.traci.constants.VAR_LANEPOSITION,
                self.traci.constants.VAR_LANE_ID
            ])

        # create the SUMO display
        self.sumoDisplay = None
        if useDisplay:
            view = self.traci.gui.getIDList()[0]
            display = self.getDisplay('sumo')
            if display is not None:
                from SumoDisplay import SumoDisplay
                self.sumoDisplay = SumoDisplay(display, displayZoom, view, directory, displayRefreshRate, displayFitSize,
                                               self.traci)

        ########################################

        risk_based = 1 #switch to gap/risk models
        #print ('Driver Behavior : ',driver_behavior,  av_arrival_time, traffic)

        ################################
        out_path = "C:\\Users\\weiminj\\Desktop\\webotoutput\\risk_based"
        
        if(risk_based==1):
            out_path = "C:\\Users\\weiminj\\Desktop\\webotoutput\\risk_based"
            print ('Risk Based Model Activated!')
        else :
            out_path = "C:\\Users\\weiminj\\Desktop\\webotoutput\\basemodel"
            print ('Gap Based Model Activated!')
            
        output = open(out_path + "\\output.txt", "w")
        
        output_TTC_Evaluate = open(out_path + "\output_TTC_Evaluate.txt", "w")
        print ("TimeStep","vehicleID","TTC ",file=output_TTC_Evaluate)
        output_TET_Evaluate = open(out_path+ "\output_TET_Evaluate.txt", "w")
        print ("TimeStep","vehicleID","TET ",file=output_TET_Evaluate)
        output_TIT_Evaluate = open(out_path+ "\output_TIT_Evaluate.txt", "w")
        print ("TimeStep","vehicleID","TIT ",file=output_TIT_Evaluate)        
        output_lead_veh = open(out_path + "\output_lead_veh.txt", "w")
        print ("TimeStep","MergingVehicleID","TTC",file=output_lead_veh)
        output_lag_veh = open(out_path + "\output_lag_veh.txt", "w")
        print ("TimeStep","MergingVehicleID","TTC",file=output_lag_veh)
        output_prob = open(out_path + "\output_prob.txt", "w")
        #output_SUMO_trajectory_profile = open(out_path + "\output_SUMO_trajectory_profile.txt", "w")
        #print("TimeStep","lane_ID_AV","Gap_AV_front","Pr_lead_prior","Pr_lag_prior","Speed_Merge","safety_distance","Acce_lag","avg_speed_target_lane","leftLeadersVehID","AV_front_vehicleID","leftFollowersVehID","Gap_lead","Gap_lag",file=output_SUMO_trajectory_profile) 
                    
        #output_lead_acc = open(out_path + "\\acc.txt","w")
                    
        # Main simulation loop
        while self.step(step) >= 0:
            if self.usePlugin:
                sumoSupervisorPlugin.run(step)

            if self.sumoDisplay is not None:
                self.sumoDisplay.step(step)

            # try to perform a SUMO step, if it fails it means SUMO has been closed by the user
            try:
                self.traci.simulationStep()
            except self.traci.exceptions.FatalTraCIError:
                print("Sumo closed")
                self.sumoClosed = True
                break

            result = self.traci.simulation.getSubscriptionResults()

            #############################
            # Mahfuz Code Starts
            #############################

            
            #print ('SUMO Time:', self.getTime())
            
            
            
            # This file are for evalaution for both baselien and risk-based model.
            av_ID="webotsVehicle0"
            segment_vehicleIDs=traci.edge.getLastStepVehicleIDs("37434591_4")
            MergingVehicleID = traci.lane.getLastStepVehicleIDs("37434591_4_0")
            print (self.count_steps*0.2,segment_vehicleIDs,TET(self.traci, self.count_steps,segment_vehicleIDs,output_TTC_Evaluate),
                   file=output_TET_Evaluate)
            print (self.count_steps*0.2,segment_vehicleIDs,TIT(self.traci, self.count_steps,segment_vehicleIDs),
                   file=output_TIT_Evaluate)                   
            try:
                TTC_merge(self.traci,self.count_steps*0.2,av_ID,output_lead_veh,output_lag_veh,MergingVehicleID)
            except Exception as err:
                pass
            
            
               
            
            #######################################
            #   RISK BASED SOLUTION
            #######################################
            if(risk_based==1):
                #This is Risk-based model
                #print ('Risk Based Model Activated!')
                output_merge_command = open(out_path + "\\output_merge_command.txt", "w")
                output_SUMO_trajectory_profile = open(out_path + "\output_SUMO_trajectory_profile.txt", "w")
                output_SUMO_lag_veh_profile=open(out_path + "\output_SUMO_lag_veh_profile.txt", "w")

                #av_ID="webotsVehicle0"


                ##################################################
                # This will tell the vehicle to stop after the merging is done.
                ##################################################
                MainRoad_1_VehicleID = self.traci.lane.getLastStepVehicleIDs("37434591_3_0")
                MergingVehicleID = self.traci.lane.getLastStepVehicleIDs("37434591_4_1")

              
                output = open(out_path + "\\output.txt", "w")

                ##################################
                #  webots vehicle
                ##################################
                try:
                    speed = self.traci.vehicle.getPosition(av_ID)
                    pos   =self.traci.vehicle.getPosition(av_ID)
                    acc  = self.traci.vehicle.getAcceleration(av_ID)

                    data = {'eid': 3, 'vid': av_ID,
                            'spd': speed,
                            'acc': acc,
                            'x': pos[0], 'y':pos[1]
                            }
                        
                    jdata = json.dumps(data)
                    #print (jdata, file=output)
                    
                        
                    
                except Exception as ex:
                    print ('could not find webots vehicle')
                    
                ##################################
                #  other vehicles
                ##################################
                for vehicle_id in MergingVehicleID:
                    speed =self.traci.vehicle.getSpeed(vehicle_id)
                    acc  = self.traci.vehicle.getAcceleration(vehicle_id)
                    pos   =self.traci.vehicle.getPosition(vehicle_id)

                    leader_vehicle = self.traci.vehicle.getLeader(vehicle_id)

                    #print (leader_vehicle)
                    
                    l_spd = 12.5
                    l_dist = 1000.0
                    if(leader_vehicle is not None):
                        l_spd = leader_vehicle[0]
                        l_dist = leader_vehicle[1]
                        
                    data = {'eid': 1, 'vid': vehicle_id,
                            'spd': speed,
                            'acc': acc,
                            'x': pos[0], 'y':pos[1],
                            'l_spd':l_spd,
                            'l_dist':l_dist}
                    
                    jdata = json.dumps(data)
                    #print (jdata, file=output)

                for vehicle_id in MainRoad_1_VehicleID:
                    speed =self.traci.vehicle.getSpeed(vehicle_id)
                    acc  =self.traci.vehicle.getAcceleration(vehicle_id)
                    pos   =self.traci.vehicle.getPosition(vehicle_id)

                    leader_vehicle = self.traci.vehicle.getLeader(vehicle_id)

                    #print (leader_vehicle)
                    
                    l_spd = 12.5
                    l_dist = 10000.0
                    if(leader_vehicle is not None):
                        l_spd = leader_vehicle[0]
                        l_dist = leader_vehicle[1]
                                            
                    data = {'eid': 1, 'vid': vehicle_id,
                            'spd': speed,
                            'acc': acc,
                            'x': pos[0], 'y':pos[1],
                            'l_spd':l_spd,
                            'l_dist':l_dist}
                    
                    jdata = json.dumps(data)
                    #print (jdata, file=output)
                    
                    
                output.close()
                ############################################
                try:
                    Speed_Merge = self.traci.vehicle.getSpeed(av_ID)
                    Acce_Merge = self.traci.vehicle.getAcceleration(av_ID)
                    Remaining_Distance = 99.49-self.traci.vehicle.getLanePosition(av_ID)
                    leftLeaders = self.traci.vehicle.getLeftLeaders(av_ID)
                    leftFollowers = self.traci.vehicle.getLeftFollowers(av_ID)
                    lane_ID_AV=self.traci.vehicle.getLaneID(av_ID) #lane ID of the front vehicle
                    Pr_lead_posterior=1
                    Pr_lag_posterior=1
                    #print("leftFollowers",leftFollowers)
                    Gap_AV_front=float("nan")
                    global AV_front_vehicleID
                    AV_front_vehicleID=""
                    ## vehicle-level evaluation
                    AV_front_vehicle=self.traci.vehicle.getLeader(av_ID)
                    if AV_front_vehicle is not None: #ensure that we have value of each variable
                        AV_front_vehicleID=AV_front_vehicle[0]
                        lane_ID_AV_front=self.traci.vehicle.getLaneID(AV_front_vehicleID) #lane ID of the front vehicle
                        Gap_AV_front=AV_front_vehicle[1]
                        speed_AV_front_veh=self.traci.vehicle.getSpeed(AV_front_vehicleID) # speed
                   
                        
                    ## system-level evaluation                    
                    target_lane_ID ="37434591_4_1"
                    avg_speed_target_lane=self.traci.lane.getLastStepMeanSpeed(target_lane_ID) # average speed of the target lane 
                    global leftLeadersVehID_global   
                    global leftFollowersVehID_global
                    leftLeadersVehID_global=""
                    leftFollowersVehID_global=""
                    print(lane_ID_AV,avg_speed_target_lane)
                    Speed_lead= float("nan")
                    Acce_lead=float("nan")
                    Gap_lead=float("nan")
                    Speed_lag= float("nan")
                    Acce_lag=float("nan")
                    Gap_lag=float("nan")
                    safety_distance=float("nan")

                    Acce_lead = 1000
                    if len(leftLeaders)>=1:
                        is_lead=True
                        
                        leftLeadersVehID_global = leftLeaders[0][0]
                        Speed_lead = self.traci.vehicle.getSpeed(leftLeadersVehID_global)
                        Acce_lead = self.traci.vehicle.getAcceleration(leftLeadersVehID_global)
                        Gap_lead=leftLeaders[0][1]
                        
                        if Gap_lead >0: 
                            Pr_lead_prior=risk_lead_prior(Acce_Merge,Remaining_Distance,Speed_lead,Acce_lead,Gap_lead)
                            Pr_gap_lead_nonconflict=lead_nonconflict_likelihood(Gap_lead)
                            Pr_gap_lead_conflict=lead_conflict_likelihood(Gap_lead)
                            Pr_lead_posterior=risk_lead_posterior(Gap_lead,Pr_lead_prior,Pr_gap_lead_nonconflict,Pr_gap_lead_conflict)
                        else:
                            Pr_lead_posterior=1
                    else:
                        is_lead =False
                        Pr_lead_posterior=0

                    Acce_lag = 1000
                    if len(leftFollowers)>=1:
                        is_lag=True
                        
                        leftFollowersVehID_global = leftFollowers[0][0]
                        Speed_lag =self.traci.vehicle.getSpeed(leftFollowersVehID_global)
                        
                        Acce_lag =self.traci.vehicle.getAcceleration(leftFollowersVehID_global)
                        Gap_lag=leftFollowers[0][1]
                        print("time","lag gap","speed_lag_veh",self.count_steps*0.2,Gap_lag+2.5,Speed_lag)
                        print(self.count_steps*0.2,Gap_lag+2.5,Speed_lag,file=output_SUMO_lag_veh_profile)
                        output_SUMO_lag_veh_profile.close()
                        
                        if Gap_lag>0:
                            Pr_lag_prior=risk_lag_prior(Speed_Merge,Acce_Merge,Remaining_Distance,Acce_lag,Gap_lag,Speed_lag-Speed_Merge)
                            Pr_gap_lag_nonconflict=lag_nonconflict_likelihood(Gap_lag)
                            Pr_gap_lag_conflict=lag_conflict_likelihood(Gap_lag)
                            Pr_lag_posterior=risk_lag_posterior(Gap_lag,Pr_lag_prior,Pr_gap_lag_nonconflict,Pr_gap_lag_conflict)
                        else:
                            Pr_lag_posterior=1
                        
                    else:
                        is_lag =False
                        Pr_lag_posterior=0
                    output_lead_acc = open(out_path + "\\acc.txt","w")
                    print (str(Acce_lead)+','+str(Acce_lag),file=output_lead_acc)
                    output_lead_acc.close()
                    
                    merge_command=0
                    # logic of risk-based merging
                    posterior_limit = 0.5
                    if (is_lead == False and is_lag ==False):
                        merge_command=1
                    elif (is_lead == True and is_lag ==False):
                        if (Pr_lead_posterior<posterior_limit):
                            merge_command=1
                            
                        else: 
                            merge_command=0
                    elif (is_lead == False and is_lag ==True):
                        if (Pr_lag_posterior<posterior_limit):
                            merge_command=1
                            
                        else: 
                            merge_command=0
                            
                    elif (is_lead == True and is_lag ==True):
                        if (Pr_lead_posterior<posterior_limit and Pr_lag_posterior<posterior_limit):
                            merge_command=1
                            
                            
                        else: 
                            merge_command=0
                            
                    print(merge_command, Pr_lead_posterior, Pr_lag_posterior, file=output_merge_command)
                    print(self.count_steps*0.2,Pr_lag_prior,Pr_lag_posterior,merge_command,file=output_prob)
                    if AV_front_vehicle is not None: #ensure that we have value of each variable
                        safety_distance=safety_distance_min(Speed_Merge,speed_AV_front_veh)
                    print(self.count_steps*0.2,lane_ID_AV,Gap_AV_front,Pr_lead_prior,Pr_lag_prior,Speed_Merge,safety_distance,Acce_lag,avg_speed_target_lane,leftLeadersVehID_global,AV_front_vehicleID,leftFollowersVehID_global,Gap_lead,Gap_lag,file=output_SUMO_trajectory_profile) 
                    output_SUMO_trajectory_profile.close()
                    #print(step,av_ID,"merge",merge_command)
                except Exception as err:
                    #print ('Exception in sumo supervisior:', sys.exc_info()[0])
                    template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                    message = template.format(type(err).__name__, err.args)
                    #print (message)
                    
                    #print (traceback.format_exc())
                    pass


                output_merge_command.close()
                

            
           



            

            else:
                #This is the baseline model
                
                MainRoad_1_VehicleID = self.traci.lane.getLastStepVehicleIDs("37434591_3_0")
                MergingVehicleID = self.traci.lane.getLastStepVehicleIDs("37434591_4_1")

              
                output = open(out_path + "\\output.txt", "w")

                ##################################
                #  webots vehicle
                ##################################
                try:
                    speed = self.traci.vehicle.getPosition(av_ID)
                    pos   =self.traci.vehicle.getPosition(av_ID)
                    acc  = self.traci.vehicle.getAcceleration(av_ID)

                    data = {'eid': 3, 'vid': av_ID,
                            'spd': speed,
                            'acc': acc,
                            'x': pos[0], 'y':pos[1]
                            }
                        
                    jdata = json.dumps(data)
                    print (jdata, file=output)
                    
                        
                    
                except Exception as ex:
                    print ('could not find webots vehicle')

                    
                for vehicle_id in MergingVehicleID:
                    speed =self.traci.vehicle.getSpeed(vehicle_id)
                    acc  =self.traci.vehicle.getAcceleration(vehicle_id)
                    pos   =self.traci.vehicle.getPosition(vehicle_id)
                    data = {'eid': 1, 'vid': vehicle_id, 'spd': speed, 'acc': acc, 'x': pos[0], 'y':pos[1]}
                    jdata = json.dumps(data)
                    print (jdata, file=output)

                for vehicle_id in MainRoad_1_VehicleID:
                    speed =self.traci.vehicle.getSpeed(vehicle_id)
                    acc  =self.traci.vehicle.getAcceleration(vehicle_id)
                    pos   =self.traci.vehicle.getPosition(vehicle_id)
                    data = {'eid': 1, 'vid': vehicle_id, 'spd': speed, 'acc': acc, 'x': pos[0], 'y':pos[1]}
                    jdata = json.dumps(data)
                    print (jdata, file=output)

                

                # for baseline model

                av_MergingVehicleID = self.traci.lane.getLastStepVehicleIDs("37434591_4_0")

                print (av_MergingVehicleID)
                
                
                webots_vehicle_id = 'webotsVehicle0'

                if(webots_vehicle_id in av_MergingVehicleID):
                    print (av_MergingVehicleID)
                    try:
                        leftLeaders =self.traci.vehicle.getLeftLeaders(webots_vehicle_id)
                        leftFollowers =self.traci.vehicle.getLeftFollowers(webots_vehicle_id)

                        av_data = {'eid':2}

                        print ('Left Leaders: ', leftLeaders)
                        print ('Left Folllowers:', leftFollowers)
                        
                        if(len(leftLeaders)>0):
                            av_data['left_leader_dist'] = leftLeaders[0][1]

                        if(len(leftFollowers)>0):
                            av_data['left_follower_dist'] = leftFollowers[0][1]

                        jdata = json.dumps(av_data)
                        print ('Main Lane status: ', jdata)
                        print (jdata, file=output)

                    except Exception as err :
                        pass #print (err)
                    
                output.close() 
        
            
	    ####################################
                        
	    #############################
            # Mahfuz Code Ends
            #############################

            
            
            # SUMO simulation over (no more vehicle are expected)
            if result[self.traci.constants.VAR_MIN_EXPECTED_VEHICLES] == 0:
                break

            # subscribe to new vehicle
            for id in result[self.traci.constants.VAR_DEPARTED_VEHICLES_IDS]:
                if not id.startswith("webotsVehicle"):
                    self.traci.vehicle.subscribe(id, self.vehicleVariableList)
                elif self.sumoDisplay is not None and len(self.webotsVehicles) == 1:
                    # Only one vehicle controlled by Webots => center the view on it
                    self.traci.gui.trackVehicle(view, 'webotsVehicle0')

            # get result from the vehicle subscription and apply it
            idList = self.traci.vehicle.getIDList()
            for id in idList:
                self.get_vehicles_position(id, self.traci.vehicle.getSubscriptionResults(id),
                                           step, xOffset, yOffset, maximumLateralSpeed, maximumAngularSpeed,
                                           laneChangeDelay)
            self.disable_unused_vehicles(idList)

            # hide unused vehicles
            self.hide_unused_vehicles()

            if not disableTrafficLight:
                for id in self.trafficLights:
                    self.update_traffic_light_state(id, self.traci.trafficlight.getSubscriptionResults(id))

            self.update_vehicles_position_and_velocity(step, rotateWheels)
            self.update_webots_vehicles(xOffset, yOffset)

            self.count_steps+=1
            

        if not self.sumoClosed:
            self.traci.close()
        else:
            self.stop_all_vehicles()

        output_TTC_Evaluate.close()
        output_TET_Evaluate.close()
        output_TIT_Evaluate.close()        
        output_lead_veh.close()
        output_lag_veh.close()
        #output_SUMO_trajectory_profile.close()
        output_prob.close()
        sys.stdout.flush()

        
