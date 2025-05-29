import csv
import os
from typing import Generic, Type, TypeVar

from DRIVE.data_types import Serializable

T = TypeVar("T", bound=Serializable)


class CsvReader(Generic[T]):
    def __init__(self, loadable_type: Type[T]):
        self.loadable_type = loadable_type
        self.data: list[T] = []

    def _from_dict(self, data: dict) -> T:
        return self.loadable_type(**data)

    def load_data_from_file(self, path: str):
        self.data = []
        filename = os.path.join(path, self.loadable_type.get_filename() + ".csv")
        with open(filename, mode="r", newline="\n", encoding="utf-8") as file:
            reader = csv.DictReader(file, fieldnames=self.loadable_type.headers())
            skip_headers = True
            for row in reader:
                if skip_headers:
                    skip_headers = False
                else:
                    self.data.append(self._from_dict(row))
