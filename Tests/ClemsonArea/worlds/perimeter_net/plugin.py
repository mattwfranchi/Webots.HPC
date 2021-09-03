import math

FPROX = 0
RPROX = 1

class SumoSupervisorPlugin:
    def __init__(self, supervisor, traci, net):
        print("Extension initializing...")
        self.voi = supervisor.getFromDef("SUMO_VEHICLE9")

        self.voi_rcam = supervisor.getFromDef("voi_rcam")


        self.voi_fdist = supervisor.getFromDef("voi_fdist")





    def run(self, ms):
        self.voi_custom_data = self.voi.getField("customData")
        self.voi_custom_data = self.voi_custom_data.getSFString()
        self.voi_fprox_flag = self.voi_custom_data[0]
        self.voi_rprox_flag = self.voi_custom_data[1]

        if self.voi_fprox_flag == '1':
            #print("Vehicle in front is too close... slowing down!")
            voi_velocity = self.voi.getVelocity()
            #print("velocity: ")
            voi_velocity[FPROX] = 0
            voi_velocity[RPROX] = 0
            self.voi.setVelocity(voi_velocity)

        if self.voi_rprox_flag == '1':
            pass
            #print("Approaching vehicle is getting close... take action!")
