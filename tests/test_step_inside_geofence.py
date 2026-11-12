import numpy as np
import pandas as pd
import pytest

import DRIVE.drive
from DRIVE.drive import Drive, ReadyState, RepositioningState, RunningState, predict_path
from DRIVE.geofencing import Geofence
from DRIVE.robot import Robot

STEP_DURATION_S = 6.0
TICK_NS = 100_000_000

# 10 m x 10 m square
SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


class SequenceSampling:
    def __init__(self, commands):
        self.commands = iter(commands)

    def sample_command(self):
        return np.array(next(self.commands))


class Setup:
    def __init__(self, tmp_path, commands, start_pose):
        self.goals = []
        self.robot = Robot(np.array(start_pose, dtype=float), lambda command: None, self.goals.append)
        self.robot.deadman_switch_pressed = True
        self.drive = Drive(self.robot, SequenceSampling(commands), 100, STEP_DURATION_S, tmp_path)
        self.drive.geofence = Geofence(SQUARE)
        self.drive.current_state = ReadyState(self.drive)  # Skip the geofence creation
        self.timestamp_ns = 0
        self.drive.start_drive(self.timestamp_ns)

    def tick(self, nb_ticks=1):
        for _ in range(nb_ticks):
            self.timestamp_ns += TICK_NS
            self.drive.run(self.timestamp_ns)

    def steps(self):
        return pd.read_csv(self.drive.dataset_recorder.datasets_folder / "steps.csv")


@pytest.fixture(autouse=True)
def no_visualization(monkeypatch):
    # stop_drive plots the dataset, which has no positions in these tests
    monkeypatch.setattr(DRIVE.drive, "generate_overview_visualization", lambda dataset: None)


def test_predict_path_straight_line():
    path = predict_path(np.array([1.0, 0.0]), np.array([1.0, 2.0, 0.0, 0.0, 0.0, np.pi / 2]), 6.0, 0.1)

    assert len(path) == 61
    np.testing.assert_allclose(path[-1], [1.0, 2.0 + 6.1], atol=1e-9)


def test_path_inside_geofence():
    geofence = Geofence(SQUARE)

    assert geofence.is_path_inside(np.array([[1.0, 1.0], [5.0, 5.0], [9.0, 9.0]]))
    assert not geofence.is_path_inside(np.array([[1.0, 1.0], [5.0, 5.0], [11.0, 5.0]]))


def test_path_inside_geofence_with_margin():
    geofence = Geofence(SQUARE)
    path_near_edge = np.array([[5.0, 5.0], [9.9, 5.0]])

    assert geofence.is_path_inside(path_near_edge)
    assert not geofence.is_path_inside(path_near_edge, margin_m=0.25)


def test_find_start_pose_fits_path():
    geofence = Geofence(SQUARE)
    local_path = np.vstack([np.zeros(2), predict_path(np.array([1.0, 0.0]), np.zeros(6), STEP_DURATION_S)])

    start_pose = geofence.find_start_pose(local_path, np.array([9.0, 5.0, 0.0, 0.0, 0.0, 0.0]))

    assert start_pose is not None
    assert geofence.is_path_inside(np.vstack([start_pose[:2], predict_path([1.0, 0.0], start_pose, STEP_DURATION_S)]))


def test_find_start_pose_none_when_path_too_long():
    geofence = Geofence(SQUARE)
    local_path = np.vstack([np.zeros(2), predict_path(np.array([5.0, 0.0]), np.zeros(6), STEP_DURATION_S)])

    assert geofence.find_start_pose(local_path, np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0])) is None


def test_step_inside_geofence_is_executed(tmp_path):
    # 1 m/s for 6 s from (2, 5) facing +x ends at (8.1, 5)
    setup = Setup(tmp_path, [[1.0, 0.0]], [2.0, 5.0, 0.0, 0.0, 0.0, 0.0])

    setup.tick()

    assert isinstance(setup.drive.current_state, RunningState)
    assert setup.drive.current_step.path_checked
    assert setup.goals == []


def test_step_leaving_geofence_repositions_robot(tmp_path):
    # From (8, 5) facing +x, the step would leave the geofence
    setup = Setup(tmp_path, [[1.0, 0.0]], [8.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    first_step_id = setup.drive.current_step.id

    setup.tick(2)  # Check the path, then send the goal
    assert isinstance(setup.drive.current_state, RepositioningState)
    assert len(setup.goals) == 1
    start_pose = setup.goals[0]

    # The robot reaches the start pose
    setup.robot.pose = start_pose
    setup.robot.goal_reached_callback()
    setup.tick(2)  # End the repositioning, then check the path again

    assert isinstance(setup.drive.current_state, RunningState)
    assert setup.drive.current_step.path_checked
    assert setup.drive.current_step.id != first_step_id
    steps = setup.steps()
    assert steps.loc[steps["id"] == first_step_id, "completion_status"].item() == "restarted"


def test_step_impossible_in_geofence_is_skipped(tmp_path):
    # 5 m/s for 6 s (30 m) cannot fit in a 10 m square, the next command can
    setup = Setup(tmp_path, [[5.0, 0.0], [1.0, 0.0]], [2.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    skipped_step_id = setup.drive.current_step.id

    setup.tick()

    assert isinstance(setup.drive.current_state, RunningState)
    assert setup.drive.current_step.command.tolist() == [1.0, 0.0]
    steps = setup.steps()
    assert steps.loc[steps["id"] == skipped_step_id, "completion_status"].item() == "skipped"


def test_step_skipped_after_max_repositionings(tmp_path):
    setup = Setup(tmp_path, [[1.0, 0.0], [1.0, 0.0]], [8.0, 5.0, 0.0, 0.0, 0.0, 0.0])

    for _ in range(DRIVE.drive.MAX_REPOSITIONINGS_PER_STEP):
        setup.tick(2)
        assert isinstance(setup.drive.current_state, RepositioningState)
        # The robot never gets to a pose where the step fits
        setup.robot.goal_reached_callback()
        setup.tick()

    setup.tick()  # Path still leaves the geofence: the step is skipped

    steps = setup.steps()
    assert steps["completion_status"].tolist()[-1] == "skipped"
    assert setup.drive.current_step.nb_repositionings == 0
