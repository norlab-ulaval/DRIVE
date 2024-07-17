#!/bin/bash

pyenv virtualenv 3.10.12 drive_env
pyenv local drive_env
pip install -r requirements.txt
source drive_env/bin/activate
