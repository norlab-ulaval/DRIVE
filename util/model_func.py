#!/usr/bin/env python

import numpy as np

# ideal differential-drive model
def diff_drive(u, k):
    # u : [Vl, Vr], k : [r, B]
    J = k[0] *  np.array([[0.5, 0.5], [0.0, 0.0], [-1 / k[1], 1 / k[1]]])
    
    x = J @ u

    return x

# extended icr-based differential-drive model (asymmetric)
def icr_diff_drive(u, k, J_icr, J_alpha):
    # u : [Vl, Vr], k : [alpha_l, alpha_r, x_icr, y_icr_l, y_icr_r]

    J_icr[0,0] = -k[4]
    J_icr[0,1] = k[3]
    J_icr[1,0] = k[2]
    J_icr[1,1] = -k[2]
    
    J_alpha[0,0] = k[0]
    J_alpha[1,1] = k[1]
    
    J = 0.3 / (k[3] - k[4]) * (J_icr @  J_alpha)
    
    x_dot = J @ u

    return x_dot

# extended icr-based differential-drive model (symmetric)
def icr_diff_drive_s(u, k, J_icr):
    # u : [Vl, Vr], k : [alpha, y_0]

    J_icr[0,0] = k[1]
    J_icr[0,1] = k[1]
    J_icr[1,0] = 0.0
    J_icr[1,1] = 0.0
    J_icr[2,0] = -1.0
    J_icr[2,1] = 1.0
    
    J = ((0.3 * k[0]) / 2 * k[1]) * (J_icr)
    
    x_dot = J @ u

    return x_dot
