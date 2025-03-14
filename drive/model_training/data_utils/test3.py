import numpy as np 



data = np.array([
    [1.0,1.0],
    [-1.0,1.0],
    [-1.0,-1.0],
    [1.0,-1.0],
])

print(np.arctan(data[:,1],data[:,0]))



