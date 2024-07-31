
import numpy as np 
import matplotlib.pyplot as plt
from norlab_controllers_msgs.action import FollowPath
from norlab_controllers_msgs.msg import PathSequence,DirectionalPath
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Pose,Quaternion,Point 
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation
#from vtkmodules.numpy_interface.dataset_adapter import numpyTovtkDataArray
class TrajectoryGenerator():

    def __init__(self) -> None:
        """Generator of 8 trajectory

        Args:
            r (_type_): radius
            entre_axe (_type_): entre-axe
        """
        
        self.x_y_trajectory = np.array([])
        self.traj_x_y_yaw = np.array([])
        self.defining_point = np.array([])
        self.rotation_matrix = np.identity(3)


    def plot_trajectory(self):
        
        fig,axs = plt.subplots(4,1)

        im = axs[0].scatter(self.x_y_trajectory[:,0],self.x_y_trajectory[:,1],c=np.arange(self.x_y_trajectory.shape[0]),label="trajectory")

        axs[0].axis("equal")
        axs[0].legend()
        axs[0].set_xlabel("X position [m]")
        axs[0].set_ylabel("Y position [m]")
        axs[0].set_title("Trajectory (x,y)")
        

        fig.colorbar(im,ax=axs[0],label="Point order")

        axs[1].scatter(np.arange(self.traj_x_y_yaw.shape[0]),self.traj_x_y_yaw[:,2])
        
        axs[1].legend()
        axs[1].set_xlabel("Position number [SI]")
        axs[1].set_ylabel("Yaw angle [rad]")
        axs[1].set_title("Trajectory angle in time")
        axs[1].set_ylim(-4,4)

        axs[2].scatter(np.arange(self.traj_x_y_yaw.shape[0]),self.traj_x_y_yaw[:,0],label="x")        
        axs[2].legend()
        axs[2].set_xlabel("Position number [SI]")
        axs[2].set_ylabel("Position x [rad]")
        axs[2].set_title("Trajectory X in time")
        

        axs[3].scatter(np.arange(self.traj_x_y_yaw.shape[0]),self.traj_x_y_yaw[:,1],label="y")
        axs[2].legend()
        axs[2].set_xlabel("Position number [SI]")
        axs[2].set_ylabel("Yaw angle [rad]")
        axs[2].set_title("Trajectory angle in time")
        

        
        plt.show()

    def adjust_number_of_lap(self,number_of_lap):
        
        list_of_lap = [self.defining_point]


        if number_of_lap>1:
            for i in range (1,number_of_lap,1):
                
                list_of_lap.append(self.defining_point[1:,:])
                
                
            self.defining_point = np.vstack(list_of_lap)
            

    def compute_trajectory_yaw(self,x_y_trajectory):

        traj_x_y_plus_yaw = np.zeros((x_y_trajectory.shape[0]+1,x_y_trajectory.shape[1]))

        traj_x_y_plus_yaw[:-1,:] = x_y_trajectory

        traj_x_y_plus_yaw[-1,:] = x_y_trajectory[0,:]


        # appending the last first point at the end to 
        # calculate the angle of the last point 
        
        next_traj = traj_x_y_plus_yaw[1:,:]
        now_traj = traj_x_y_plus_yaw[0:-1,:]

        diff_traj = next_traj - now_traj

        yaw = np.arctan2(diff_traj[:,1],diff_traj[:,0])


        traj_x_y_yaw = np.zeros((x_y_trajectory.shape[0],x_y_trajectory.shape[1]+1))
        traj_x_y_yaw[:,:2] = x_y_trajectory
        traj_x_y_yaw[:,2] = yaw

        self.traj_x_y_yaw = traj_x_y_yaw    

        return traj_x_y_yaw    
        
    def find_sharp_angle(self,traj_x_y_yaw,degre_seuil=70):
        vector = np.diff(traj_x_y_yaw,axis=0)
        a = vector[:-1,:2]
        b = vector[1:,:2]

        a_norm = np.linalg.norm(a,axis=1)
        b_norm = np.linalg.norm(b,axis=1)

        dot_roduct_result = np.diag(a@b.T)

        to_aracos = dot_roduct_result /(a_norm * b_norm)

        angle = np.arccos(to_aracos)

        seuil = np.deg2rad(degre_seuil)

        angle_mask = np.where(angle >seuil,np.ones(angle.shape[0]),np.zeros(angle.shape[0]))

        nbr_of_sharp_angle = int(np.sum(angle_mask))

        nbr_final_point = traj_x_y_yaw.shape[0] + nbr_of_sharp_angle

        final_traj = np.zeros([nbr_final_point,3])

        i = 0
        nbr_coin = 0
        print("\n"*3,str(angle_mask))
        while i < (nbr_final_point):
            
            final_traj[i,:] = traj_x_y_yaw[i-nbr_coin,:]
            i += 1
            if i-nbr_coin-1 < angle_mask.shape[0]: # if that point points to a corner
                if angle_mask[i-nbr_coin-1]:
                    point_precedent = traj_x_y_yaw[i-nbr_coin-1,:2]
                    point_now = traj_x_y_yaw[i-nbr_coin,:2]
                    unit_vector = (point_now-point_precedent)/np.linalg.norm((point_now-point_precedent))

                    transform_2d = np.zeros((3,3)) 
                    transform_2d[2,2] =1
                    final_traj[i,:2] = traj_x_y_yaw[i-nbr_coin,:2]
                    final_traj[i,2] = traj_x_y_yaw[i-nbr_coin-1,2] # Prend l'angle duprécédent
                    nbr_coin += 1
                    i += 1
        
        
        return final_traj

    def export_2_norlab_controller(self,time_stamp,frame_id,transform_2d,forward=True):
        """Export the 8

        Args:
            time_stamp (_type_): _description_
            frame_id (_type_): _description_
            forward (bool, optional): _description_. Defaults to True.

        Returns:
            _type_: _description_
        """
        
        header = Header()
        header.frame_id = frame_id
        header.stamp = time_stamp
        trajectory_length = self.traj_x_y_yaw.shape[0]

        print("\n" *3, self.rotation_matrix )
        transform_2d = transform_2d @ self.rotation_matrix
        

        traj_x_y_rel_8_homo = np.ones((trajectory_length,3))
        traj_x_y_rel_8_homo[:,:2] = self.traj_x_y_yaw[:,:2]
        traj_x_y_rel_map = transform_2d @ traj_x_y_rel_8_homo.T

        
        #print("\n"*3,traj_x_y_rel_map- traj_x_y_rel_8_homo.T,"\n"*3)
        final_traj_in_abs = self.compute_trajectory_yaw(traj_x_y_rel_map[:2,:].T)
        final_traj_in_abs = self.find_sharp_angle(final_traj_in_abs)
        # Traj new length
        trajectory_length = final_traj_in_abs.shape[0]
        list_posetamped = [PoseStamped() for i in range(trajectory_length)]
        z_increment = 0.5
        for i in range(trajectory_length):
            
            point = final_traj_in_abs[i,:]

            point_ros = Point()
            point_ros.x, point_ros.y, point_ros.z = point[0], point[1], z_increment * 1

            point_rot_scipy = Rotation.from_euler("zyx",[point[2],0,0],degrees=False)
            #print("euleur z",point[2])
            #print("quaternion",point_rot_scipy.as_quat())
            orientation_ros = Quaternion()
            orientation_ros.x, orientation_ros.y, orientation_ros.z, orientation_ros.w = point_rot_scipy.as_quat()
            
            pose_ros = Pose()
            pose_ros.position = point_ros
            pose_ros.orientation = orientation_ros

            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = pose_ros

            list_posetamped[i] = pose_stamped


    
        # Create the Directionnal Path 
        direct_path_ros = DirectionalPath()
        direct_path_ros.header = header
        direct_path_ros.poses = list_posetamped
        direct_path_ros.forward = forward

        # Create the Path message for laying out the display in foxglove
        visualize_path_ros = Path()
        visualize_path_ros.header = header
        visualize_path_ros.poses = list_posetamped
        
        # Create Path sequence 
        path_sequence = PathSequence()
        path_sequence.header = header
        path_sequence.paths = [direct_path_ros]
        # Create the Path sequence
        return path_sequence,visualize_path_ros
            

