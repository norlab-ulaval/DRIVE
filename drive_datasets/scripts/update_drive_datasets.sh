#!/bin/bash
cd ../../drive/model_training/data_utils

# Activate the Pyenv environment named 'data_analysis'
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init --path)"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

pyenv activate data_analysis



python3 update_drive_inventory.py