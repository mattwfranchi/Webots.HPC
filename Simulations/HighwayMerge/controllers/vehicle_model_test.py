import numpy as np
import matplotlib.pyplot as plt
import random

def vehicle_model(prev_state, dt, pedal, steering):
    x_t = prev_state[0]
    y_t = prev_state[1]
    psi_t = prev_state[2]
    v_t = prev_state[3]
        
    beta = steering
    a_t = pedal
    #v_dot = a_t
     
    x_dot = v_t*np.cos(psi_t)
    y_dot = v_t*np.sin(psi_t)
     
    psi_dot = v_t* np.tan(beta)/2.1
    v_dot = a_t
        
        
    x_t += x_dot*dt
    y_t -= y_dot*dt
    psi_t += psi_dot*dt
    v_t += v_dot*dt

    return [x_t, y_t, psi_t, v_t]


x = 62
y = 35

state = [x,y,0,10]
states = []

for i in range(100):
    
    state = vehicle_model(state, 0.1, 1, 0.25)

    states.append(state)
    
    #print (state)


states = np.array(states)

#print (states)

#print (states[:,0])

plt.plot(states[:,0],states[:,1])

plt.show()
