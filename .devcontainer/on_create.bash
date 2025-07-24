#!/bin/bash

sudo chown -R $(whoami) /home/ws/
pip install -e .
sudo apt-get update -y
sudo rosdep update
sudo rosdep install --from-paths ros --ignore-src -y
