import scipy
import math
import numpy as np
from scipy.optimize import minimize

class MPC():
    def __init__(self):
        self.horizon = 10
        self.dt = 0.3
        self.n = 3

        # refernces or set points for the controller.
        self.y_ref = 2         #m
        self.speed_limit = 22 #m/s
        self.cte_ref = 0

        self.w_params = [[1,100,5000,1],
                         [1,1,5500,1],
                         [1,100,150,1]]


        # vehicle params

        #lane params

        p1 = np.array([0.25,34.47])
        p2 = np.array([-82.1, 60.90])

        p3 = np.array([6.25,36.4])
        p4 = np.array([-85.6, 65.5])

        #p1 = np.array([-.76, 33.42])
        #p2 = np.array([-94.03, 63.93])

        #p3 = np.array([-1.88, 36.94])
        #p4 = np.array([-92.89, 67.46])

        self.lane_xy = [(p1,p2),(p3,p4)]

        self.lane_id = 1

        self.lf = 4.5
        # constraints
        self.bounds = []
        self.set_bounds()

        self.u = np.ones(self.horizon*self.n)

    #def normalize(x):
    def get_weights(self,b):
        return self.w_params[b]

    def vehicle_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        b = prev_state[4]

        beta = steering
        a_t = pedal
        #v_dot = a_t

        x_t = x_t - v_t*np.sin(psi_t)*dt
        y_t = y_t + v_t*np.cos(psi_t)*dt

        #psi_t = np.arctan(y_dot,x_dot)

        psi_t = psi_t + (v_t*beta*dt)/self.lf

        v_t = v_t + a_t*dt

        #psi_dot = v_t * np.tan(beta)/self.lf
        #psi_dot = v_t * beta/self.lf
        #v_dot = a_t*dt


        #x_t = x_t + x_dot*dt
        #y_t = y_t + y_dot*dt
        #psi_t += psi_dot*dt
        #v_t += v_dot*dt

        return [x_t, y_t, psi_t, v_t,b]

    def cost(self,u,*args):
        state = args[0]

        ##print (args)

        #ref = args[1]
        cost = 0
        inti_state = state
        for k in range(0,self.horizon):
            x_t = state[0]
            y_t = state[1]
            psi_t = state[2] #np.arctan(y_t/x_t) #state[2]
            v_t = state[3]
            b = state[4]
            #state[2] = psi_t #np.arctan(y_t,x_t)

            #psi_t = np.arctan(y_t-x_t)
            #dist = state[4]

            ##print (k , ' : ', k**2+1)
            prev_state = state
            state = self.vehicle_model(state,self.dt, u[k*2], u[k*2+1])

            ws = self.get_weights(b)
            # cost 1
            cost += ws[0]*(state[3] - self.speed_limit)**2
            cost += 100*(state[3] - prev_state[3])**2
            # cost 2
            cost += ws[1]*(self.cte(self.lane_id,state[0],state[1])-self.cte_ref)**2

            #cost += (state[1] - self.y_ref)**2
            #cost += state[1]*state[3]



            #cost += (state[1] - prev_state[1])**2

            #cost 3
            cost += ws[2] * (state[2] - prev_state[2])**2

            #cost #4
            #cost += ws[3] * state[2]**2

            #cost += 10 * (state[1] - prev_state[1])**2
            #cost += 10 * (state[0] - prev_state[0])**2

            #cost += ws[2] * (state[2] - prev_state[2])**2

            #cost += 50* (state[1] - prev_state[1])**2


            #dist_cost = self.dist_cost(state[0],state[1])
            #cost+= dist_cost

            #cost+= 10*state[3]


            #cost += np.absolute(state[2])*np.absolute(state[3])

        return cost

    def cte(self,lane_id, x,y):
        p3 = np.array([x,y])
        p_ref = self.lane_xy[self.lane_id]
        p1, p2 = p_ref[0], p_ref[1]
        d = abs(np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1))
        ##print (d)
        return d

    def dist_cost(self,x,y):
        dist = (x+50)**2 + (y-2.5)**2

        if(dist>20):
            return 20
        else: return 1000.0/(30*dist)

    def set_bounds(self):
        for i in range(self.horizon*self.n):
            self.bounds+=[[-1,1]] # acc
            #self.bounds+=[[-1,1]] # steering

    def run(self,cur_state):

        # current state contains the following info:
        # index 0 : x pos
        # index 1 : y_pos
        # index 2 : psi
        # index 3 : speed
        # index 4 : state
        # index 5 : acc

        np.delete(self.u,0)
        np.delete(self.u,0)

        np.append(self.u,self.u[-2])
        np.append(self.u,self.u[-2])

        u_sol = minimize(self.cost,
                        self.u,
                        (cur_state,0),
                        method='SLSQP',
                        bounds = self.bounds,
                        tol = 1e-8)

        self.u = u_sol.x

        ##print (u_sol)


        ##print (u_sol.x)

        return self.u
