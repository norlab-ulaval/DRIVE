import matplotlib.pyplot as plt 
import numpy as np
import matplotlib as mpl
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

plt.rc('font', **font)
plot_fs = 12

plt.rc('font', family='serif', serif='Times')
plt.rc('text', usetex=True)
plt.rc('xtick', labelsize=9)
plt.rc('ytick', labelsize=9)
plt.rc('axes', labelsize=10)
mpl.rcParams['lines.dashed_pattern'] = [2, 2]
mpl.rcParams['lines.linewidth'] = 1.0

def add_vehicle_limits_to_wheel_speed_graph(ax):
        
        max_wheel_speed = 16.6667 # Les roues decluches. rad/s
        ax.vlines(np.array([-max_wheel_speed,max_wheel_speed]),ymin=-max_wheel_speed,ymax=max_wheel_speed,color="black",label="vehicle limits")
        ax.hlines(np.array([-max_wheel_speed,max_wheel_speed]),xmin=-max_wheel_speed,xmax=max_wheel_speed,color="black")

        ax.plot(np.array([-max_wheel_speed,max_wheel_speed]),np.array([-max_wheel_speed,max_wheel_speed]),label="r = inf :straight line",color="black",ls="--")
        ax.plot(np.array([-max_wheel_speed,max_wheel_speed]),np.array([max_wheel_speed,-max_wheel_speed]),label="r = 0 : Turning on spot",color="black",ls="dotted")

        x_array = np.array([-max_wheel_speed,0,max_wheel_speed,0])
        y_array = np.array([0,-max_wheel_speed,0,max_wheel_speed])
        s_array = [r"$r=\frac{-b}{2}$", r"$r=\frac{b}{2}$",r"$r=$$\frac{-b}{2}$",r"$r=\frac{b}{2}$"]

        repetition = 5
        offset = -1
        x_array = np.linspace(0,0,repetition)+offset
        y_array = np.linspace(-max_wheel_speed,0,repetition)
        s_array = [r"$\frac{-b}{2}$"]*repetition
        #for x,y,s in zip(x_array,y_array,s_array):
        #    ax.text(x,y,s)\frac{b}{2}

        repetition=3
        x = np.linspace(-max_wheel_speed,0,repetition)
        y2 = np.linspace(-max_wheel_speed,0,repetition)
        y1 = np.zeros(x.shape)
        #ax.text(x[1],y2[1]*3/2,r"$r>\frac{b}{2}$")
        #ax.text(x[1]*3/2,y2[1]*1/2,r"$r<\frac{-b}{2}$")
        ax.fill_betweenx(x,y1,y2,label=r"$\frac{b}{2}<r<inf$",color="blue",alpha=0.1)
        
        
        
        
        x2 = np.linspace(0,max_wheel_speed,repetition)
        
        y4 = np.zeros(x.shape)
        y3 = np.ones(x.shape) * -max_wheel_speed
        
        ax.fill_between(x2,y3,y4,label=r"$\frac{-b}{2}<r<\frac{b}{2}$",color="red",alpha=0.1)
        ax.text(x2[1]-2,y3[1]/2,r"$r=0$")
        ax.text(-x2[1]-2,-y3[1]/2,r"$r=0$")
        ax.text(-2,2-max_wheel_speed,r"$r=\frac{b}{2}$")
        ax.text(-2,-2+max_wheel_speed,r"$r=\frac{b}{2}$")
        ax.text(-6+max_wheel_speed,-0.5,r"$r=\frac{-b}{2}$")
        ax.text(+6-max_wheel_speed,-0.5,r"$r=\frac{-b}{2}$")
        
        ax.fill_between(x,y2,0,label=r"$-inf<r<\frac{-b}{2}$",color="yellow",alpha=0.1)
        
        
        ax.fill_between(-x2,-y3,y4,color="red",alpha=0.1)
        x = np.linspace(0,max_wheel_speed,repetition)
        
        
        ax.fill_between(x,x,max_wheel_speed,color="blue",alpha=0.1)
        
        ax.fill_between(x,x,0,color="yellow",alpha=0.1)
        
        ax.legend(ncols=2,bbox_to_anchor=(0.08, 1))

        ax.set_ylabel("left_wheel speed [rad/s]")
        ax.set_xlabel("right wheel speed [rad/s]")
            
        return ax

fig, ax = plt.subplots(1,1)
fig.set_figwidth(5)
fig.set_figheight(5)

ax =add_vehicle_limits_to_wheel_speed_graph(ax)
test=1

plt.show()