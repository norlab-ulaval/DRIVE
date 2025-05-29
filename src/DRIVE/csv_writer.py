import csv
import os
from dataclasses import asdict
from typing import Generic, Type, TypeVar

from DRIVE.data_types import Serializable

T = TypeVar("T", bound=Serializable)


class CsvWriter(Generic[T]):
    def __init__(self, saveable_type: Type[T]):
        self.saveable_type = saveable_type
        self.data: list[T] = []

    def save_line(self, line: T):
        if not isinstance(line, self.saveable_type):
            raise TypeError(f"Expected type {self.saveable_type.__name__}, got {type(line).__name__}")
        self.data.append(line)

    def save_data_to_file(self, path: str):
        filename = os.path.join(path, self.saveable_type.get_filename() + ".csv")
        with open(filename, mode="w", newline="\n", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=self.saveable_type.headers())
            writer.writeheader()

            for row in self.data:
                writer.writerow(asdict(row))
