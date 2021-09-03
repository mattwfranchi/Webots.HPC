import numpy as np


p1 = np.array([1.689097,33.64849827])
p2 = np.array([-85.3136, 61.37882604])

p3 = np.array([2.814350386,37.04438674])
p4 = np.array([-85.28607818, 65.03660994])

lane_xy = [(p1,p2),(p3,p4)]

#p3 = np.array([34.8143775, -82.33344691])
#p3 = np.array([34.81435821, -82.33352041])
#p3 = np.array([34.81431098, -82.3335742])
p3 = np.array([-4.004671222, 35.52612991])

p3= np.array([-59.98240897, 57.28428551])

d= abs(np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1))

#print (d)


def cte(lane_id, p3):
    p_ref = lane_xy[lane_id]
    p1, p2 = p_ref[0], p_ref[1]
    d = abs(np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1))
    return d

# point on lane 1
p3 = np.array([-82.9886972, 64.34750368])
print (cte(0,p3))

#point on lane 0
p3= np.array([-78.68226814, 59.37106661])
print (cte(1,p3))


