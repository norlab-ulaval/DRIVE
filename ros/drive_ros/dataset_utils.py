import datetime
import os


def resolve_dataset_directory(datasets_root: str) -> str:
    """Return a new ``<timestamp>/node_data`` directory in ``datasets_root`` for this run.

    Every launch gets its own folder, so a previous run is never reused. If a folder with the same
    timestamp already exists (two launches in the same second), a numeric suffix is added.
    """
    os.makedirs(datasets_root, exist_ok=True)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = os.path.join(datasets_root, stamp)
    suffix = 1
    while os.path.exists(run_folder):
        run_folder = os.path.join(datasets_root, f"{stamp}_{suffix}")
        suffix += 1

    return os.path.join(run_folder, "node_data")
