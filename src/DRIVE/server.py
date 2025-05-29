import base64
import io
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO
from matplotlib.figure import Figure

from DRIVE.common import Pose
from DRIVE.drive import Drive, GeofenceCreationState
from DRIVE.plot import draw_input_space, draw_robot_visualization_figure

WHEEL_BASE = 1.5


class Server:
    def __init__(self, drive: Drive, get_current_timestamp_ns: Callable[[], float]):
        self.app, self.socketio = self.create_server(drive, get_current_timestamp_ns)

        self.drive = drive
        self.fig_viz, self.ax_viz = plt.subplots()
        self.fig_input_space, self.ax_input_space = plt.subplots()

    def run(self):
        self.socketio.run(self.app, host="0.0.0.0", debug=False, allow_unsafe_werkzeug=True, use_reloader=False)

    def update_visualization(self):
        geofence_points = self.drive.get_geofence_points()

        self.update_robot_viz(self.drive.robot.pose, geofence_points, WHEEL_BASE)
        self.update_input_space(self.drive.get_commands())

        if self.drive.can_skip_command():
            self.skippable_state_start()
        else:
            self.skippable_state_end()

    def encode_fig_to_b64(self, fig: Figure):
        buffer = io.BytesIO()
        fig.savefig(buffer, format="png")
        return base64.b64encode(buffer.getvalue()).decode("utf-8")

    def update_robot_viz(self, robot_pose: Pose, geofence_points: np.ndarray, wheel_base: float):
        draw_robot_visualization_figure(self.ax_viz, robot_pose, geofence_points, wheel_base)
        viz_b64 = self.encode_fig_to_b64(self.fig_viz)

        self.socketio.emit("robot_vizualisation_update", {"image_data": viz_b64})

    def update_input_space(self, commands: np.ndarray):
        draw_input_space(self.ax_input_space, commands)
        input_space_b64 = self.encode_fig_to_b64(self.fig_input_space)

        self.socketio.emit("input_space_update", {"image_data": input_space_b64})

    def skippable_state_start(self):
        self.socketio.emit("skippable_state_start")

    def skippable_state_end(self):
        self.socketio.emit("skippable_state_end")

    def state_transition(self, state: str):
        self.socketio.emit("state_transition", state)

    def create_server(self, drive: Drive, get_current_timestamp_ns: Callable[[], float]):
        app = Flask(__name__, static_url_path="/static", static_folder="web/static", template_folder="web/templates")
        socketio = SocketIO(app)

        @app.route("/")
        def index():
            return render_template("index.html")

        @socketio.on("start_drive")
        def start_drive():
            current_time = get_current_timestamp_ns()
            # For now doing both at the same time
            if drive.current_state.__class__ == GeofenceCreationState:
                drive.confirm_geofence(current_time)
            drive.start_drive(current_time + 1)

        @socketio.on("start_geofencing")
        def start_geofencing():
            current_time = get_current_timestamp_ns()
            drive.start_geofence(current_time)

        @socketio.on("save_dataset")
        def save_dataset(data):
            dataset_name = data.get("name")
            drive.save_dataset(dataset_name)

        @socketio.on("update_datasets")
        def update_datasets():
            self.socketio.emit("datasets", drive.get_datasets())

        @socketio.on("load_geofence")
        def load_geofence(data):
            current_time = get_current_timestamp_ns()
            dataset_name = data.get("name")
            drive.load_geofence(dataset_name, current_time)

        @socketio.on("skip_command")
        def skip_command():
            current_time = get_current_timestamp_ns()
            drive.skip_current_step(current_time)

        @socketio.on("stop_drive")
        def stop_drive(data):
            current_time = get_current_timestamp_ns()
            reason = data.get("reason")
            drive.stop_drive(reason, current_time)

        return app, socketio
