import numpy as np 


test = np.array([[1,1,1],[2,2,2],[3,3,3]])


list = [test]
list.append(test[1:,:]*10)
print(np.vstack(list))

