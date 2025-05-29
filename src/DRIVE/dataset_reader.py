import os

from DRIVE.csv_reader import CsvReader
from DRIVE.data_types import GeofencePoint, Serializable


class DatasetReader:
    def __init__(self, datasets_folder: str):
        self.datasets_folder = datasets_folder
        self.step_id = 0

        self.readers: dict[type[Serializable], CsvReader] = {}

        # Built-in recorders
        self._register(GeofencePoint)

    def _register(self, saveable_type: type[Serializable]):
        self.readers[saveable_type] = CsvReader(saveable_type)

    def get_datasets(self) -> list[str]:
        return os.listdir(self.datasets_folder)

    def load_geofence(self, dataset_name: str) -> list[GeofencePoint]:
        folder_path = os.path.join(self.datasets_folder, dataset_name)

        self.readers[GeofencePoint].load_data_from_file(folder_path)

        return self.readers[GeofencePoint].data
