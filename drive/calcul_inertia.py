import numpy as np

def compute_space_between_gear():
    m = 3 # modulus
    z_1 =  20# number of teeths
    z_2 = 10 # number of teeth

    entre_ax = m/2 * (z_1 + z_2)

    print(f"Entre_axe = {np.round(entre_ax,4)} mm")


n = 1050

print(np.rad2deg(np.arctan(1/2)))
print(np.sqrt(3200))

print(41.904 -37.560)

print(18.272-19.104)


masse = 67 * 1/1000 # kg

masse_lbs = 0.14771
volume = 33540.2283 # mm3
print(f'densite {masse/volume}')#mm?3


inertie_grande_roue_dente_lbs_mm = 13821.24972 # lbmm2 453.592
inertie_grande_roue_dente_kg_mm = inertie_grande_roue_dente_lbs_mm * 0.45359 #kg/lbs
inertie_grande_roue_dente_kg_m = inertie_grande_roue_dente_kg_mm * (1/100)**2 #Kg m**2 

# Torque = J acceleration

# Acceleration_max = Torque/J
maximum_torque = 2.5 * 10 /100
acceleration_max = inertie_grande_roue_dente_kg_m / maximum_torque

print(acceleration_max)

print((1.76-2.6)*180)