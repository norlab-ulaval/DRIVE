import open3d
from scipy.spatial.transform import Rotation
import numpy as np

rotation = Rotation.from_euler("xyz",[0,0,np.pi])

rotation_quat = rotation.as_quat(scalar_first=False)

print(rotation_quat)

print(open3d.geometry.get_rotation_matrix_from_quaternion(rotation_quat))

print(rotation.as_matrix())