import os
import json


class Utils:
    def __init__(self, path=None, data=None):
        self.path = path
        self.data = data

    def load_file(self, path):
        if os.path.exists(path):
            with open(path, "r") as file:
                return json.load(file)
        return []

    def save_file(self, data, filepath):

        directory = os.path.dirname(filepath)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)

        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
    
