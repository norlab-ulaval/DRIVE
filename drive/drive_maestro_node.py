import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, Int32
from drive_custom_srv.srv import BashToPath
from std_srvs.srv import Empty
class DriveMaestroNode(Node):
    """
    Class that sends out commands to the nodes for a full step by step drive experiment
    """
    # def __init__(self, max_lin_speed, min_lin_speed, lin_speed_step, max_ang_speed, ang_steps,
    #              step_len, dead_man_button, dead_man_index, dead_man_threshold, ramp_trigger_button, ramp_trigger_index,
    #              calib_trigger_button, calib_trigger_index, response_model_window, steady_state_std_dev_threshold,
    #              cmd_rate_param, encoder_rate_param):
    def __init__(self):
        super().__init__('drive_maestro_node')

        self.path_saved = False


        self.drive_maestro_operator_action_msg = String() #Create Topic for operator action
        self.drive_maestro_operator_action_msg.data = "Drive around the perimeter of the area on which you want to do the drive experiment. Once done, click map done"#init at chill

        self.drive_maestro_status_msg = String()
        self.drive_maestro_status_msg.data = "map_construction" #init at mapping

        self.drive_maestro_path_to_drive_folder_msg = String()
        self.drive_maestro_path_to_drive_folder_msg.data = "folder path will appear here once drive is over" # TODO get the path of the experiment name


        self.drive_maestro_operator_action_pub = self.create_publisher(String, 'drive_maestro/operator_action', 10)
        self.drive_maestro_status_pub = self.create_publisher(String, 'drive_maestro/status', 10)
        self.drive_maestro_path_to_drive_folder_pub = self.create_publisher(String, 'drive_maestro/experiment_data_path', 10)

        self.operator_action_listener = self.create_subscription(String,'operator_action_drive', self.drive_node_operator_action_callback,1000)

        timer_period = 0.5  # seconds #TIMER
        self.timer = self.create_timer(timer_period, self.timer_callback) #TIMER execute callback

        self.srv = self.create_service(BashToPath, 'path_to_folder', self.log_path) #serice for saving data path
        self.srv = self.create_service(BashToPath, 'drive_status', self.log_drive) #service for starting drive
    
    def timer_callback(self):
        self.publish_drive_maestro_operator_action()
        self.publish_drive_maestro_status()
        self.publish_drive_maestro_path_to_drive_folder()

    
    #TOPIC SUBSCRIBED
    
    
    def drive_node_operator_action_callback(self, msg): #operator action FROM drive node
        self.drive_maestro_operator_action_msg = msg

    
    #TOPIC PUBLISH
    
    def publish_drive_maestro_operator_action(self): # Operator action
        self.drive_maestro_operator_action_pub.publish(self.drive_maestro_operator_action_msg)

    def publish_drive_maestro_status(self): # Status
        self.drive_maestro_status_pub.publish(self.drive_maestro_status_msg)

    def publish_drive_maestro_path_to_drive_folder(self): # Path
        self.drive_maestro_path_to_drive_folder_pub.publish(self.drive_maestro_path_to_drive_folder_msg)

    
    #SEVICES
    def log_path(self, request, response):
        self.drive_maestro_path_to_drive_folder_msg.data = request.input
        response.output = 'path published to topic: experiment_data_path'
        self.drive_maestro_status_msg.data = 'training model'
        self.drive_maestro_operator_action_msg.data = 'Drive experiment is finished, wait for model training to finish before starting next step...'
        return response

    def log_drive(self, request, response):
        self.drive_maestro_operator_action_msg.data = request.input
        self.drive_maestro_status_msg.data = 'drive_ready'
        self.drive_maestro_operator_action_msg.data = "Press characterization trigger to start drive"
        return response





def main():
    rclpy.init()
    drive_maestro = DriveMaestroNode()
    rclpy.spin(drive_maestro)
    rclpy.shutdown()


if __name__ == '__main__':
    main()