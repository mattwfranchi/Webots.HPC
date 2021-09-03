from controller import Supervisor
import sys

TIME_STEP = 32

supervisor = Supervisor() 

voi = supervisor.getFromDef("SUMO_VEHICLE3")
voi_rcam = supervisor.getFromDef("REAR_CAMERA")
voi_fdist = supervisor.getFromDef("FRONT_DISTANCE_SENSOR")

if not voi: 
    sys.stderr.write("NO DEF SUMO_VEHICLE3 node found in the current world file")
    sys.exit(1)


if voi_rcam: 
    voi_rcam.enable()

if voi_fdist: 
    voi_fdist.enable()
    

while supervisor.step(TIME_STEP) != -1: 
    voi_rcam_frame = voi_rcam.getImage()
    voi_fdist_val = voi_fdist.getValue()



voi_rcam.disable()
voi_fdist.disable()