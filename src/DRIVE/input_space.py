from matplotlib import pyplot as plt
import numpy as np
import shapely

from DRIVE.models import WARTHOG_MODEL, IdealDiffDriveModel


class InputSpace:
    def __init__(
        self,
        model: IdealDiffDriveModel,
        min_linear_speed,
        max_linear_speed,
        min_angular_speed,
        max_angular_speed,
        min_wheel_speed,
        max_wheel_speed,
    ):
        self.model = model

        self.body_frame_constraints = np.array(
            [
                [min_linear_speed, max_angular_speed],
                [max_linear_speed, max_angular_speed],
                [max_linear_speed, min_angular_speed],
                [min_linear_speed, min_angular_speed],
            ]
        )

        self.actuator_constraints = np.array(
            [
                [max_wheel_speed, min_wheel_speed],
                [max_wheel_speed, max_wheel_speed],
                [min_wheel_speed, max_wheel_speed],
                [min_wheel_speed, min_wheel_speed],
            ]
        )

    def body_input_space(
        self,
    ) -> shapely.Polygon:
        body_frame_wheel_constraints = self.model.matrix_forward_kinematics(self.actuator_constraints)

        polygon_body_constraints = shapely.Polygon(self.body_frame_constraints)
        polygon_actuator_constraints = shapely.Polygon(body_frame_wheel_constraints)

        polygon_body_input_space: shapely.Polygon = shapely.intersection(polygon_actuator_constraints, polygon_body_constraints)  # type: ignore

        return polygon_body_input_space

    def actuator_input_space(self) -> shapely.Polygon:
        wheel_frame_body_constraints = self.model.matrix_inverse_kinematics(self.body_frame_constraints)

        polygon_body_constraints = shapely.Polygon(wheel_frame_body_constraints)
        polygon_actuator_constraints = shapely.Polygon(self.actuator_constraints)

        polygon_actuator_input_space: shapely.Polygon = shapely.intersection(polygon_body_constraints, polygon_actuator_constraints)  # type: ignore

        return polygon_actuator_input_space


if __name__ == "__main__":
    model = WARTHOG_MODEL

    # Body constraints
    min_linear_speed = -0.5
    max_linear_speed = 0.5
    min_angular_speed = -0.2
    max_angular_speed = 0.2

    # Actuator constraints
    min_wheel_speed = -13.33
    max_wheel_speed = 13.33

    input_space = InputSpace(
        model,
        min_linear_speed,
        max_linear_speed,
        min_angular_speed,
        max_angular_speed,
        min_wheel_speed,
        max_wheel_speed,
    )

    body_input_space = input_space.body_input_space()
    actuator_input_space = input_space.actuator_input_space()

    fig, (ax_body, ax_wheel) = plt.subplots(2, 1, figsize=(10, 10))

    ax_body.set_title("Body Input Space")
    x, y = body_input_space.exterior.xy
    ax_body.plot(y, x, color="red", label="Body Input Space")
    ax_body.axis("equal")
    ax_body.set_xlabel("Angular Speed (rad/s)")
    ax_body.set_ylabel("Linear Speed (m/s)")

    ax_wheel.set_title("Actuator Input Space")
    x, y = actuator_input_space.exterior.xy
    ax_wheel.plot(y, x, color="blue", label="Actuator Input Space")
    ax_wheel.axis("equal")
    ax_wheel.set_xlabel("Right Wheel Speed (rad/s)")
    ax_wheel.set_ylabel("Left Wheel Speed (rad/s)")

    plt.show()
