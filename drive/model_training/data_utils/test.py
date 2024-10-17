import numpy as np
import pandas as pd 

data= {"test":[1,2,3,4,5,6,7,8,9,10],"test2":[1,2,3,4,5,6,7,8,9,10],"test3":[1,2,3,4,5,6,7,8,9,10]}


df = pd.DataFrame.from_dict(data)


print(np.max(df[["test","test3"]],axis=1))