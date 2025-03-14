#!/bin/bash

echo ""
echo ""
echo "Here is the list of your pyenv venv"
pyenv versions
echo ""

echo "checking if the drive_env exists"
# Check if drive_env exists and delete it if it does
#if pyenv virtualenvs | grep -q "drive_env"; then
#    echo "Delete the pyenv drive env"
#    pyenv uninstall -f drive_env
    
#fi

echo ""
echo "Here is a grep of drive_env."
pyenv versions | grep drive_env
echo ""

echo " Creating the drive_env"

pyenv virtualenv 3.10.12 drive_env
pyenv local drive_env
pip install -r requirements.txt
source drive_env/bin/activate

echo ""
echo "Here is a grep of drive_env."
pyenv versions | grep drive_env


