import numpy as np

def quaternion_pose_to_transform(pose, transform):
    """ A function that modifies a rigid transformation array based on a specified pose

    :param pose: pose array consisting of [x, y, z, quat_w, quat_x, quat_y, quat_z]
    :param transform: rigid transformation array that should be changed
    """

    transform[0:3, 3] = pose[0:3]

    quat_wx = pose[3] * pose[4]
    quat_wy = pose[3] * pose[5]
    quat_wz = pose[3] * pose[6]
    quat_xx = pose[4] * pose[4]
    quat_xy = pose[4] * pose[5]
    quat_xz = pose[4] * pose[6]
    quat_yy = pose[5] * pose[5]
    quat_yz = pose[5] * pose[6]
    quat_zz = pose[6] * pose[6]

    transform[0,0] = 1.0-2.0*(quat_yy + quat_zz)
    transform[0,1] = 2.0*(quat_xy - quat_wz)
    transform[0,2] = 2.0*(quat_xz - quat_wy)
    transform[1,0] = 2.0*(quat_xy + quat_wz)
    transform[1,1] = 1.0 - 2.0*(quat_xx + quat_zz)
    transform[1,2] = 2.0*(quat_xz - quat_wx)
    transform[2,0] = 2.0*(quat_xz - quat_wy)
    transform[2,1] = 2.0*(quat_yz + quat_wx)
    transform[2,2] = 1.0 - 2.0*(quat_xx + quat_yy)

    return True

def euler_to_rotmat(euler):
    R = np.empty((3,3))
    s_roll = np.sin(euler[0])
    c_roll = np.cos(euler[0])
    s_pitch = np.sin(euler[1])
    c_pitch = np.cos(euler[1])
    s_yaw = np.sin(euler[2])
    c_yaw = np.cos(euler[2])

    R[0,0] = c_pitch * c_yaw
    R[0, 1] = c_yaw * s_pitch * s_roll - c_roll * s_yaw
    R[0, 2] = s_roll * s_yaw + c_roll * c_yaw * s_pitch
    R[1, 0] = c_pitch * s_yaw
    R[1, 1] = c_roll * c_yaw + s_pitch * s_roll * s_yaw
    R[1, 2] = c_roll * s_pitch * s_yaw - c_yaw * s_roll
    R[2, 0] = -s_pitch
    R[2, 1] = c_pitch * s_roll
    R[2, 2] = c_pitch * c_roll
    return R

def yaw_to_rotmat2d(R, yaw):
    s_yaw = np.sin(yaw)
    c_yaw = np.cos(yaw)
    R[0,0] = c_yaw
    R[0, 1] = -s_yaw
    R[1,0] = s_yaw
    R[1,1] = c_yaw
    return True

def euler_to_transform(euler_angles, transform):
    transform[0:3, 0:3] = euler_to_rotmat(euler_angles)
    return transform

def euler_pose_to_transform(euler_angles, position, transform):
    transform[0:3, 0:3] = euler_to_rotmat(euler_angles)
    transform[0:3, 3] = position
    return transform


def cross_product_skew_symmetric_from_vector(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])

def euler_pose_to_omega_submatrix(euler_angles, transform):
    """ Equation defined in equation A.14, A.15 (Appendix A) of Seegmiller thesis. Allows to convert from angular velocity to Euler rate angles

    :param euler_angles: Set of euler angles [roll, pitch, yaw]
    :param transform: 3x3 identity matrix in which to enter the values
    :return:
    """
    s_roll = np.sin(euler_angles[0])
    c_roll = np.cos(euler_angles[0])
    s_pitch = np.sin(euler_angles[1])
    c_pitch = np.cos(euler_angles[1])
    t_pitch = np.tan(euler_angles[1])

    transform[0,1] = s_roll * t_pitch
    transform[0,2] = c_roll * t_pitch
    transform[1,1] = c_roll
    transform[1,2] = -s_roll
    transform[2,1] = s_roll / c_pitch
    transform[2,2] = c_roll / c_pitch
