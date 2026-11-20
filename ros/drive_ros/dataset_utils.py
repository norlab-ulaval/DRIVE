import datetime
import os


def resolve_dataset_directory(datasets_root: str) -> str:
    """Return the ``node_data`` directory where the DRIVE node should save this run.

    The most recent folder in ``datasets_root`` is reused only if its ``node_data`` is missing or empty,
    meaning it was just created for this run (e.g. by a rosbag recorder started before the node).
    Otherwise, a new timestamped folder is used so a previous run is never reused.
    """
    os.makedirs(datasets_root, exist_ok=True)

    folders = [d for d in os.listdir(datasets_root) if os.path.isdir(os.path.join(datasets_root, d))]
    if folders:
        latest = max(folders, key=lambda d: os.path.getmtime(os.path.join(datasets_root, d)))
        node_data = os.path.join(datasets_root, latest, "node_data")
        if not os.path.isdir(node_data) or not os.listdir(node_data):
            return node_data

    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return os.path.join(datasets_root, stamp, "node_data")
