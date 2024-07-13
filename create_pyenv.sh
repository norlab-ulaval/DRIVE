#!/bin/bash

pyenv virtualenv 3.10 drive_env
pyenv local drive_env
pip install pandas
pip install numpy 1.22.4
pip install pathlib
pip install scipy
pip install torch