import numpy as np
import pytest

import DRIVE.drive
from DRIVE.drive import Drive, ReadyState
from DRIVE.robot import Robot

STEP_DURATION_NS = 1_000_000_000


class FixedSampling:
    def sample_command(self):
        return np.array([1.0, 0.0])


def create_drive(dataset_directory, target_nb_steps=3):
    robot = Robot(np.zeros(6), lambda command: None, lambda goal: None)
    drive = Drive(robot, FixedSampling(), target_nb_steps, 1.0, dataset_directory)
    drive.current_state = ReadyState(drive)  # Skip the geofence creation
    return drive


def run_steps(drive, timestamp_ns, statuses):
    """Start a drive, end one step per status, then stop the drive."""
    drive.start_drive(timestamp_ns)
    for status in statuses:
        timestamp_ns += STEP_DURATION_NS
        drive.sample_next_step(timestamp_ns, status)
    drive.stop_drive(timestamp_ns)
    return timestamp_ns


@pytest.fixture(autouse=True)
def no_visualization(monkeypatch):
    # stop_drive plots the dataset, which has no positions in these tests
    monkeypatch.setattr(DRIVE.drive, "generate_overview_visualization", lambda dataset: None)


def test_total_starts_at_zero_on_new_dataset(tmp_path):
    drive = create_drive(tmp_path)

    assert drive.total_nb_steps_completed == 0


def test_total_counts_only_completed_steps(tmp_path):
    drive = create_drive(tmp_path)
    drive.start_drive(0)

    for timestamp_ns, status in zip([1, 2, 3, 4], ["completed", "skipped", "completed", "restarted"]):
        drive.sample_next_step(timestamp_ns * STEP_DURATION_NS, status)

    assert len(drive.completed_commands) == 2
    assert drive.total_nb_steps_completed == 2


def test_total_is_kept_when_drive_is_stopped(tmp_path):
    drive = create_drive(tmp_path)

    timestamp_ns = run_steps(drive, 0, ["completed", "completed"])
    assert len(drive.completed_commands) == 0
    assert drive.total_nb_steps_completed == 2

    run_steps(drive, timestamp_ns, ["completed", "completed", "completed"])
    assert drive.total_nb_steps_completed == 5


def test_total_is_read_from_existing_dataset(tmp_path):
    run_steps(create_drive(tmp_path), 0, ["completed", "skipped", "completed", "completed"])

    restarted_drive = create_drive(tmp_path)

    assert restarted_drive.total_nb_steps_completed == 3
