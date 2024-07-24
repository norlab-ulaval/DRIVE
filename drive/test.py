import yaml 
import pandas as pd
import numpy as np



vector = np.diff(traj_x_y,axis=0)
a = vector[:-1,:]
b = vector[1:,:]

a_norm = np.linalg.norm(a,axis=1)
b_norm = np.linalg.norm(b,axis=1)

dot_roduct_result = np.diag(a@b.T)

to_aracos = dot_roduct_result /(a_norm * b_norm)

angle = np.arccos(to_aracos)

seuil = np.deg2rad(70)

angle_mask = np.where(angle >seuil,np.ones(angle.shape[0]),np.zeros(angle.shape[0]))

nbr_of_sharp_angle = int(np.sum(angle_mask))

nbr_final_point = traj_x_y.shape[0] + nbr_of_sharp_angle

final_traj = np.zeros([nbr_final_point,2])

i = 0
nbr_coin = 0

while i < (nbr_final_point):
    
    final_traj[i,:] = traj_x_y[i-nbr_coin,:]
    i += 1
    if i-nbr_coin < angle_mask.shape[0]:
        if angle_mask[i-nbr_coin]:
            final_traj[i,:2] = traj_x_y[i-nbr_coin,:2]
            final_traj[i,2] = traj_x_y[i-nbr_coin-1,:2] # Prend l'angle duprécédent

print(angle_mask)