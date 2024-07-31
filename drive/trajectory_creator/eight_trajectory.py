import numpy as np 
import matplotlib.pyplot as plt
from norlab_controllers_msgs.action import FollowPath
from norlab_controllers_msgs.msg import PathSequence,DirectionalPath
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Pose,Quaternion,Point 
from scipy.spatial.transform import Rotation
from drive.trajectory_creator.trajectory_creator import TrajectoryGenerator
#from vtkmodules.numpy_interface.dataset_adapter import numpyTovtkDataArray
from nav_msgs.msg import Path

class EightTrajectoryGenerator(TrajectoryGenerator):

    def __init__(self,r,entre_axe,horizon) -> None:
        self.rotation_matrix = np.identity(3)
        self.r = r
        self.entre_axe = entre_axe
        self.horizon = horizon
        self.centers = np.array([[0,entre_axe/2],[0,-entre_axe/2]])

        
    def calculate_defining_angle(self):
        self.defining_angle = np.arccos(2*self.r/ self.entre_axe)

        rotation_angle = self.defining_angle #np.pi/2 - 
        rotation = Rotation.from_euler("zxy",[rotation_angle,0,0],degrees=False)
        self.rotation_matrix = rotation.as_matrix() 


    def calculate_section_point(self):
        """Calculates the 5 points that divid linear interpolation from circular interpolation.
        """
        self.defining_point = np.zeros((7,3)) # the third columns is for linear (0) or circular interpolation (1)

        l = self.r  * np.tan(self.defining_angle)

        x1 = l * np.cos(self.defining_angle)
        y1 = l * np.sin(self.defining_angle)

        self.defining_point[1,:] = np.array([x1,y1,1])
        self.defining_point[2,:] = np.array([-x1,y1,0]) # centre du cercle
        # we pass two time by 0,0 
        self.defining_point[4,:] = np.array([x1,-y1,-1])
        self.defining_point[5,:] = np.array([-x1,-y1,0]) # centre du cercle
        # We finish at the begining

        

    def interpolate_point(self):
        """Calculates interpolates to put one point each two meters
        """

        nb_interpolation = self.defining_point.shape[0] -1

        nb_points_final = np.array([0,0]).reshape(1,2)
        cercle_center_id = 0
        for i in range(nb_interpolation):
            start = self.defining_point[i,:2]
            end = self.defining_point[i+1,:2]
            interpolation_type = self.defining_point[i,2]

            print(f"yeah {start},{end},{interpolation_type}")
            if interpolation_type == 0: #Linéaire
                #1 calculate le nb de points
                norm = np.linalg.norm(end-start) 
                nb_points = int(np.ceil(norm/self.horizon)) 
                # calculate the multiplicators
                real_horizon = norm/nb_points
                unit_vector = (end-start)/norm
                #print(unit_vector)
                #print(start)
                #print(end)
                multiplicator = np.arange(nb_points).reshape((nb_points,1)) * real_horizon
                x_y = multiplicator * unit_vector.reshape([1,2]) + start

                

            elif interpolation_type == 1 or interpolation_type == -1: # Circulare sens horaire ou antihoraire
                total_angle_of_rotation = np.pi + 2 * (np.pi/2 - self.defining_angle)
                distance_2_travel = self.r * (total_angle_of_rotation)
                
                nb_points = int(np.ceil(distance_2_travel/self.horizon))
                angle_increment = total_angle_of_rotation/nb_points

                real_horizon = distance_2_travel/nb_points
                
                if interpolation_type ==1:
                    start_angle = - (np.pi/2 - self.defining_angle)
                elif interpolation_type == -1:
                    start_angle = + (np.pi/2 - self.defining_angle) +2* np.pi
                x_y = np.zeros((nb_points,2))

                center_coordinate = self.centers[cercle_center_id,:] 
                for i in range(nb_points):
                    x_y[i,:] = np.array([np.cos(start_angle),np.sin(start_angle)]) * self.r + center_coordinate
                    if interpolation_type==1:
                        start_angle += angle_increment
                    elif interpolation_type == -1:
                        start_angle-= angle_increment
                
                if cercle_center_id ==1:
                    cercle_center_id = 0
                elif cercle_center_id ==0:
                    cercle_center_id = 1
                                
                                
            
            nb_points_final = np.vstack((nb_points_final,x_y))

        
        self.x_y_trajectory = nb_points_final[1:,:] # to remove the poit to initialize



    def plot_important_point(self):
        
        fig,axs = plt.subplots(1,1)

        axs.scatter(self.centers[:,0],self.centers[:,1],label="centers")
        
        axs.scatter(self.defining_point[:,0],self.defining_point[:,1],label="defining_points")
        axs.axis("equal")
        axs.legend()
        plt.show()

        
    def compute_trajectory(self,number_of_laps=1):

        self.calculate_defining_angle()
        
        self.calculate_section_point()
        self.adjust_number_of_lap(number_of_laps)
        self.interpolate_point()
        self.compute_trajectory_yaw(self.x_y_trajectory)
        #self.plot_trajectory()
        


class RectangleTrajectoryGenerator(TrajectoryGenerator):

    def __init__(self,width,length,horizon) -> None:
        self.width = width
        self.length = length
        self.horizon = horizon
        self.rotation_matrix = np.identity(3)
        
        
    def calculate_section_point(self):
        """Creates a rectangle where the robot is on the bottom left corner going towards the bottom right corner
        """
        self.defining_point = np.zeros((5,3)) # the third columns is for linear (0) or circular interpolation (1)

        self.defining_point[1,:] = np.array([self.length,0,0])
        self.defining_point[2,:] = np.array([self.length,self.width,0]) # centre du cercle
        # we pass two time by 0,0 
        self.defining_point[3,:] = np.array([0,self.width,0])
        # We finish at the begining

        

    def interpolate_point(self):
        """Calculates interpolates to put one point each two meters
        """

        nb_interpolation = self.defining_point.shape[0] -1

        nb_points_final = np.array([0,0]).reshape(1,2)
        cercle_center_id = 0
        for i in range(nb_interpolation):
            start = self.defining_point[i,:2]
            end = self.defining_point[i+1,:2]
            interpolation_type = self.defining_point[i,2]

            print(f"yeah {start},{end},{interpolation_type}")
            if interpolation_type == 0: #Linéaire
                #1 calculate le nb de points
                norm = np.linalg.norm(end-start) 
                nb_points = int(np.ceil(norm/self.horizon)) 
                # calculate the multiplicators
                real_horizon = norm/nb_points
                unit_vector = (end-start)/norm
                #print(unit_vector)
                #print(start)
                #print(end)
                multiplicator = np.arange(nb_points).reshape((nb_points,1)) * real_horizon
                x_y = multiplicator * unit_vector.reshape([1,2]) + start

                

            elif interpolation_type == 1 or interpolation_type == -1: # Circulare sens horaire ou antihoraire
                total_angle_of_rotation = np.pi + 2 * (np.pi/2 - self.defining_angle)
                distance_2_travel = self.r * (total_angle_of_rotation)
                
                nb_points = int(np.ceil(distance_2_travel/self.horizon))
                angle_increment = total_angle_of_rotation/nb_points

                real_horizon = distance_2_travel/nb_points
                
                if interpolation_type ==1:
                    start_angle = - (np.pi/2 - self.defining_angle)
                elif interpolation_type == -1:
                    start_angle = + (np.pi/2 - self.defining_angle) +2* np.pi
                x_y = np.zeros((nb_points,2))

                center_coordinate = self.centers[cercle_center_id,:] 
                for i in range(nb_points):
                    x_y[i,:] = np.array([np.cos(start_angle),np.sin(start_angle)]) * self.r + center_coordinate
                    if interpolation_type==1:
                        start_angle += angle_increment
                    elif interpolation_type == -1:
                        start_angle-= angle_increment
                if cercle_center_id ==1:
                    cercle_center_id = 0
                elif cercle_center_id ==0:
                    cercle_center_id = 1
                                
            
            nb_points_final = np.vstack((nb_points_final,x_y))

        self.x_y_trajectory = nb_points_final[1:,:] # to remove the poit to initialize




    def plot_important_point(self):
        
        fig,axs = plt.subplots(1,1)

        axs.scatter(self.centers[:,0],self.centers[:,1],label="centers")
        
        axs.scatter(self.defining_point[:,0],self.defining_point[:,1],label="defining_points")
        axs.axis("equal")
        axs.legend()
        plt.show()

        
    def compute_trajectory(self,number_of_laps=1):

        self.calculate_section_point()
        self.adjust_number_of_lap(number_of_laps)
        self.interpolate_point()
        self.compute_trajectory_yaw(self.x_y_trajectory)
        #self.plot_trajectory()
        

    

        
if __name__=="__main__":
    traj = EightTrajectoryGenerator(10,100,2)

    traj.compute_trajectory(number_of_laps=2)
    #traj.export_2_norlab_controller("test","test")
    traj.plot_trajectory()
    plt.show()

    traj = RectangleTrajectoryGenerator(10,100,2)

    traj.compute_trajectory(number_of_laps=2)
    traj.plot_trajectory()
    #traj.export_2_norlab_controller("test","test")

    

