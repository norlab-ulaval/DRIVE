import numpy as np 
import sympy as sp 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D





wl, wr, vx, b, r = sp.symbols("wl wr vx b r")
expr = sp.Eq(wl/(r+b/2), vx/r)

sp.pprint(expr)

radius_eq = sp.Eq(sp.solve(expr,r)[0],r)

turning_left_radius_without_vx = sp.simplify(radius_eq.subs(vx, (wr-wl)/2))


turning_right = sp.Eq(sp.solve( sp.Eq(wr/(r+b/2), vx/r),r)[0],r)
turning_right_radius_without_vx = sp.simplify(turning_right.subs(vx, (wl-wr)/2))





sp.pprint(turning_left_radius_without_vx)
sp.pprint(turning_right_radius_without_vx)


piecewise_func = sp.Piecewise(
    (turning_left_radius_without_vx.rhs, wr > wl),     # x^2 for x < 0
    (turning_right_radius_without_vx.rhs, wr < wl),
    (10000, wl==wr)    # x + 1 for x >= 0
)

print("Radius function")
sp.pprint(piecewise_func)
radius_function = sp.lambdify((wl,wr,b),piecewise_func,"numpy")

def compute_radi(wl,wr,b):

    if wl>0 and wr >0 :

        if wl>= wr:
            r = b*(wl+wr)/(2*(wl-wr))
        elif wr >=wl:
            r = b*(wl+wr)/(2*(wl-wr))

    elif wl<0 and wr<0:
        if wl>= wr:
            r = b*(wl+wr)/(2*(wl-wr))
        elif wr >=wl:
            r = b*(wl+wr)/(2*(wl-wr))
    elif wl > 0 and wr <0:
        r = b*(wl+wr)/(2*(wl-wr))
        #if abs(wl) > abs(wr):
        #    r = b*(wl+wr)/(2*(wl-wr))
        #else:
        #    
        #    r = b * (wl+wr)/(2*(3*wl+wr))
    elif wl < 0 and wr >0:
        
        r = b*(wl+wr)/(2*(wl-wr))
        #if abs(wl) > abs(wr):
        #    r = b*(wl+wr)/(2*(wl-wr))
        #else:
        #    r = b * (wl+wr)/(2*(3*wl+wr))

    return r





def computing_turning_radi(wl_array,wr_array,b=1.08):
    r_array = np.zeros(wl_array.shape)
    for i in range(wl_array.shape[0]):
        wl = wl_array[i]
        wr = wr_array[i]
        r_array[i] = compute_radi(wl,wr,b)

    return r_array



    


#### ARRay

n_points = 100

x_range = np.linspace(-20, 20, n_points)  # 5 points from -1 to 1
y_range = np.linspace(-20, 20, n_points)


left_wheel, right_wheel = np.meshgrid(x_range, y_range)

left_wheel_raveled= np.ravel(left_wheel)
right_wheel_raveled = np.ravel(right_wheel)

b_array = np.ones(left_wheel_raveled.shape) *1.08


radius = computing_turning_radi(left_wheel_raveled,right_wheel_raveled,b=1.08)#radius_function(left_wheel_raveled,right_wheel_raveled,b_array)

color_axis = np.where(radius<0,["red"]*radius.shape[0],["blue"]*radius.shape[0])

radius_limit = 10
mask = np.where(np.abs(radius) <radius_limit, True,False )

Z_masked = np.ma.masked_outside(radius.reshape(left_wheel.shape), -2, 2)

#### Graphs 



# Generate random data for the scatter plot

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
ax.scatter(right_wheel_raveled,left_wheel_raveled, radius, c=color_axis, marker='o')

# Labels and title
ax.set_xlabel('right_wheel vel')
ax.set_ylabel('left_wheel vel')
ax.set_zlabel('turning radius')
ax.set_title('3D Scatter Plot')
ax.set_zlim(-3,3)
# Show the plot
plt.show()


###


plt.figure(figsize=(8, 6))
contour = plt.contour(right_wheel_raveled.reshape(right_wheel.shape), left_wheel_raveled.reshape(left_wheel.shape), Z_masked, levels=25, cmap='viridis')  # Change levels as needed
plt.clabel(contour)  # Optional: Label the contours
ax.set_xlabel('right_wheel vel')
ax.set_ylabel('left_wheel vel')
ax.set_zlabel('turning radius')

plt.colorbar(label='Radius of curvature')
plt.show()