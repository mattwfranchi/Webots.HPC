import pandas as pd
from controller import Robot, Camera, DistanceSensor

TIME_STEP = 100
ELAPSED = 0

FPROX = 0
RPROX = 1

FPROX_VAL = 400.0
RPROX_VAL = 600.0

columnNames = ["Front Proximity Flag", "Rear Proximity Flag", "# Cars Front", "# Cars Right", "# Cars Back", "# Cars Left"]
data = pd.DataFrame(columns = columnNames)

voi = Robot()

voi_rcam = voi.getDevice("voi_rcam")
if voi_rcam is not None: 
    voi_rcam.enable(TIME_STEP)
else:
    print("Rear camera sensor not found")    

voi_fdist = voi.getDevice("voi_fdist")
if voi_fdist is not None: 
    voi_fdist.enable(TIME_STEP)
else:
    print("Front distance sensor not found")

voi_rdist = voi.getDevice("voi_rdist")
if voi_rdist is not None: 
    voi_rdist.enable(TIME_STEP)
else:
    print("Rear distance sensor not found")

voi_radar_front = voi.getDevice("voi_radar_front")
if voi_radar_front is not None: 
    voi_radar_front.enable(TIME_STEP)
else:
    print("Front radar sensor not found")

voi_radar_rside = voi.getDevice("voi_radar_rside")
if voi_radar_rside is not None:
    voi_radar_rside.enable(TIME_STEP)
else:
    print("Right side radar sensor not found")

voi_radar_back = voi.getDevice("voi_radar_back")
if voi_radar_back is not None:
    voi_radar_back.enable(TIME_STEP)
else:
    print("Back radar sensor not found")

voi_radar_lside = voi.getDevice("voi_radar_lside")
if voi_radar_lside is not None: 
    voi_radar_lside.enable(TIME_STEP)
else:
    print("Left side radar sensor not found")

while voi.step(TIME_STEP) != 1:
    # Getting sensor values
    voi_rcam_frame = voi_rcam.getImage()
    voi_fdist_val = voi_fdist.getValue()
    voi_rdist_val = voi_rdist.getValue()
     
   
    voi_custom_data = voi.getCustomData()
    voi_custom_data_list = list(voi_custom_data)
    
    voi_radar_front_ncars = voi_radar_front.getNumberOfTargets()
    voi_radar_rside_ncars = voi_radar_rside.getNumberOfTargets()
    voi_radar_back_ncars = voi_radar_back.getNumberOfTargets() 
    voi_radar_lside_ncars = voi_radar_lside.getNumberOfTargets() 
    print("Vehicles detected [F R B L]: " + str(voi_radar_front_ncars) + " " + str(voi_radar_rside_ncars) + " " + str(voi_radar_back_ncars) + " " 
    + str(voi_radar_lside_ncars)) 
    
    
    # Checking for front proximity flag
    if voi_fdist_val < FPROX_VAL: 
        #print("Front distance value is: " + str(voi_fdist_val))
        voi_custom_data_list[FPROX] = '1'
    else:
        voi_custom_data_list[FPROX] = '0'
        
    if voi_rdist_val < RPROX_VAL:
        #print("Rear distance value is: " + str(voi_rdist_val))
        voi_custom_data_list[RPROX] = '1'
    else:
        voi_custom_data_list[RPROX] = '0'
    
    
        
    # Updating custom data
    voi_custom_data = ''.join(voi_custom_data_list)
    voi.setCustomData(voi_custom_data) 
        
    # Updating flags
    voi_fprox_flag = voi_custom_data[FPROX]
    voi_rprox_flag = voi_custom_data[RPROX]
    
    
    # Adding data to dataframe
    data.loc[ELAPSED] = [voi_fprox_flag, voi_rprox_flag, voi_radar_front_ncars, voi_radar_rside_ncars, voi_radar_back_ncars, voi_radar_lside_ncars]
    
    
    # Updating elapsed time
    ELAPSED += TIME_STEP
    
    


data.to_csv(r"C:\Users\Matt\OneDrive - Clemson University\Clemson University\Research\SUMO\Tests\ClemsonArea\worlds\data.csv")
        
voi_rcam.disable()
voi_fdist.disable()

voi_radar_front.disable()
voi_radar_rside.disable()
voi_radar_back.disable()
voi_radar_lside.disable()







