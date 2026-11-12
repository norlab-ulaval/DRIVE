from DRIVE.dataset_recorder import DatasetRecorder
from DRIVE.writing import DriveStep


def test_count_completed_steps_without_steps_file(tmp_path):
    recorder = DatasetRecorder(tmp_path)

    assert recorder.count_completed_steps() == 0


def test_count_completed_steps_counts_only_completed(tmp_path):
    recorder = DatasetRecorder(tmp_path)
    statuses = ["completed", "skipped", "completed", "restarted", "completed"]
    recorder.append_multiple([DriveStep(i, i, i + 1, 1.0, 0.0, status) for i, status in enumerate(statuses)])

    assert recorder.count_completed_steps() == 3
