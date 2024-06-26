import pandas as pd
import matplotlib.pyplot as plt

if __name__ == "main":
    path = "/home/robot/data/drive/test.pkl"
    
    df = pd.read_pickle(path)
    
    print(df.columns)
    print(df.head(5))
    fig, axs = plt.subplots(1,1)
    y = df.meas_left_vel.astype(float)  
    x = df.ros_time.astype(float)  
    axs.plot(x.values,y.values)
    
    plt.show()

