from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt


#Twist msg
twist_x = 3.545
twist_theta_z = 0.342523525

#icp_odom_initial_state
X_init = 30.0
Y_init = 15.0
theta_z_init = 10.0

#matrice_init = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
#matrice_init.astype(float)

#
mat_from_quat = R.from_quat([0, 0, 0, 1])
matrice_init= mat_from_quat.as_matrix()

matrice_init[0, 2] = X_init
matrice_init[1, 2] = Y_init

print(matrice_init)

total_time = 6.0 #TODO
nb_pas_de_temps = 20*int(total_time)
delta_s = total_time/nb_pas_de_temps
delta_x= twist_x*delta_s #m/s
delta_z_dot = twist_theta_z*delta_s #rad/s
matrice_commande = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
matrice_commande.astype(float)
matrice_commande[0, 0] = np.cos(delta_z_dot)
matrice_commande[0, 1] = -np.sin(delta_z_dot)
matrice_commande[1, 0] = np.sin(delta_z_dot)
matrice_commande[1, 1] = np.cos(delta_z_dot)
matrice_commande[0, 2] = delta_x
ptsx = []
ptsy = []
#print(matrice_commande)

for i in range (nb_pas_de_temps):
    matrice_init = matrice_init @ matrice_commande
    ptsx.append(matrice_init[0, 2])
    ptsy.append(matrice_init[1, 2]) 

print(matrice_init)
fig = plt.figure()
ax = fig.add_subplot()
# square plot
ax.set_aspect('equal', adjustable='box')
plt.scatter(ptsx, ptsy)
plt.show()

print(int(0.424349))