from rclpy.node import Node
import logging


def declare_parameter_from_dataclass(node: Node, dataclass_inst):
    for field_name in dataclass_inst.__dataclass_fields__.keys():
        default_value = dataclass_inst.__getattribute__(field_name)

        node.declare_parameter(field_name, default_value)

    # Make sure we update the values if the default are overwritten with a config/launch file
    update_parameter_from_dataclass(node, dataclass_inst)


def update_parameter_from_dataclass(node: Node, dataclass_inst):
    for field_name in dataclass_inst.__dataclass_fields__.keys():
        value = node.get_parameter(field_name).value

        dataclass_inst.__setattr__(field_name, value)


def redirect_logging_to_ros2(node: Node):
    ros2_logger = node.get_logger()

    class ROS2Handler(logging.Handler):
        def emit(self, record):
            log_entry = self.format(record)
            if record.levelno == logging.DEBUG:
                ros2_logger.debug(log_entry)
            elif record.levelno == logging.INFO:
                ros2_logger.info(log_entry)
            elif record.levelno == logging.WARNING:
                ros2_logger.warn(log_entry)
            elif record.levelno == logging.ERROR:
                ros2_logger.error(log_entry)
            elif record.levelno == logging.CRITICAL:
                ros2_logger.fatal(log_entry)

    root_logger = logging.getLogger()
    for handler in root_logger.handlers:
        root_logger.removeHandler(handler)

    ros2_handler = ROS2Handler()
    formatter = logging.Formatter("%(levelname)s: %(message)s")
    ros2_handler.setFormatter(formatter)
    root_logger.addHandler(ros2_handler)
    root_logger.setLevel(logging.INFO)
