import numpy as np

def transform_index(i):
    range = (-np.pi/2, np.pi/2)
    len_laser_scan = 5.0
    angle = i*range[1]/(len_laser_scan-1) + range[0] - i*range[0]/(len_laser_scan-1) 
    return angle


print("transform index 0: {a}".format( a = np.rad2deg( transform_index(0.0)) ) )
print("transform index 1: {a}".format( a = np.rad2deg( transform_index(1.0)) ) )
print("transform index 2: {a}".format( a = np.rad2deg( transform_index(6.0)) ) )