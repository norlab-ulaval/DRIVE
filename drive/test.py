from scipy.spatial.transform import Rotation
import numpy as np 


test_sie = 5
yaw = np.linspace(0,2*np.pi,test_sie)


rpy = np.zeros((test_sie,3))

rpy[:,2] = yaw

print(rpy)


rotations = Rotation.from_euler("xyz",rpy)

print(rotations)