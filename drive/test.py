from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter,ParameterValue
from rclpy.parameter import ParameterMsg
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType



# Create a Parameter message
parameter = Parameter()
parameter.name = 'example_parameter'

# Set the parameter value type to integer
parameter.value.type = ParameterType.PARAMETER_DOUBLE
parameter.value.double_value = 42.0



req = SetParameters.Request()

req.parameters= [parameter]