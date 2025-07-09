from rclpy.node import Node


def declare_parameter_from_dataclass(node: Node, dataclass_inst):
    for field_name in dataclass_inst.__dataclass_fields__.keys():
        default_value = dataclass_inst.__getattribute__(field_name)

        node.declare_parameter(field_name, default_value)


def update_parameter_from_dataclass(node: Node, dataclass_inst):
    for field_name in dataclass_inst.__dataclass_fields__.keys():
        value = node.get_parameter(field_name).value

        dataclass_inst.__setattr__(field_name, value)
