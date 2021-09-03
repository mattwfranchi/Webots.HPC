from risk_functions import *
import os, os.path

###########################################
#
###########################################
proj_cwd = os.getcwd()
proj_cwd = os.path.abspath(__file__+'/../../../..')

def read_acc_file():
    led_acc  = 0.0
    lag_acc =  0.0
    file_name = proj_cwd+"\\acc.txt"
    if os.path.isfile(file_name):
        with open(file_name,"r") as f:
            for line in f:
                led_acc , lag_acc = line.split(',')
                print (led_acc, lag_acc)

    return [float(led_acc) , float(lag_acc)]

def change_lane_on_risk(radar_b,radar_f,radar_l,v_state,r_dist):
    output_risk_profile = open(proj_cwd+"\\output_crash_risk.txt", "w")
    lead_vehicle_dist = 10000.0
    lag_vehicle_dist = 10000.0

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
            return False # a vehicle is on the bliend spot so do not lane change.


    ####################################
    # process the back radar
    #####################################

    n_targets =  radar_b.getNumberOfTargets()
    targets = radar_b.getTargets()

    blind_spot = False
    is_lag = False

    for i in range(n_targets):
        dist =  targets[i].distance
        speed = targets[i].speed
        azi = targets[i].azimuth
        K = dist*azi
        #print("K",K)

        #print ('Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6), ' theta:', targets[i].azimuth )

        if(K>0 and K<5):
            if(dist<lag_vehicle_dist):
              lag_vehicle_dist = dist
              lag_vehicle_spd = v_state[3] - speed
              #print("lag_vehicle_dist","lag_vehicle_spd",lag_vehicle_dist,lag_vehicle_spd)
              #lag_veh_profile(lag_vehicle_dist,lag_vehicle_spd)   #read
              is_lag = True
          #return False

    ####################################
    # process the front radar
    #####################################
    n_targets =  radar_f.getNumberOfTargets()
    targets = radar_f.getTargets()
    is_lead = False
    for i in range(n_targets):
        dist =  targets[i].distance
        speed = targets[i].speed
        azi = targets[i].azimuth
        K = dist*azi

        #print ('Factor:', (K),'  Distance : ', targets[i].distance, ' speed : ',(targets[i].speed*3.6+driver.getCurrentSpeed()), ' theta:', targets[i].azimuth )

        if(K > -3.5 and K < -2): # and dist>2 and dist<GAP_LAG):
            if(dist<lead_vehicle_dist):
              lead_vehicle_dist = dist
              lead_vehicle_spd = v_state[3] - speed
              is_lead = True


    # now that I got the lead vehile and lag vehiucle distance measure the probability.
    print ('lead vehicle dist: ', lead_vehicle_dist, ' lag vehicle distance:',lag_vehicle_dist)
    # lead vehicls spd, lead vehicle distance
    # lag vehicls spd,  lag vehicle distance
    lead_acc = 0.0
    lag_acc = 0.0
    Pr_lead_posterior=1
    Pr_lag_posterior=1

    if(is_lead or is_lag):
        lead_acc , lag_acc = read_acc_file()
        print ('lead acc :', lead_acc, ' lag acc : ', lag_acc)

    if(is_lead == True):
        Pr_lead_prior=risk_lead_prior(v_state[5],r_dist,lead_vehicle_spd,lead_acc,lead_vehicle_dist) # acc of lead vehicle.
        Pr_gap_lead_nonconflict=lead_nonconflict_likelihood(lead_vehicle_dist)
        Pr_gap_lead_conflict=lead_conflict_likelihood(lead_vehicle_dist)
        Pr_lead_posterior=risk_lead_posterior(lead_vehicle_dist,Pr_lead_prior,Pr_gap_lead_nonconflict,Pr_gap_lead_conflict)
    else:
        Pr_lead_posterior = 0

    if (is_lag== True):
        print("lag_vehicle_dist","lag_vehicle_spd",lag_vehicle_dist,lag_vehicle_spd)
        Pr_lag_prior=risk_lag_prior(v_state[3],v_state[5],r_dist,lag_acc,lag_vehicle_dist,lag_vehicle_spd-v_state[3]) #1.0 acc of lag vehicle
        Pr_gap_lag_nonconflict=lag_nonconflict_likelihood(lag_vehicle_dist)
        Pr_gap_lag_conflict=lag_conflict_likelihood(lag_vehicle_dist)
        Pr_lag_posterior=risk_lag_posterior(lag_vehicle_dist,Pr_lag_prior,Pr_gap_lag_nonconflict,Pr_gap_lag_conflict)
    else:
        Pr_lag_posterior= 0
    print("crash risk",Pr_lead_posterior,Pr_lag_posterior)
    print(Pr_lead_posterior,Pr_lag_posterior,file=output_risk_profile)
    output_risk_profile.close()
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

    if(merge_command==1): return True
    else: return False
