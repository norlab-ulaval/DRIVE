#!/bin/bash
# Check if the correct argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 produce_video=true|false"
    exit 1
fi

# Extract the value of produce_video from the argument
if [[ $1 == produce_video=* ]]; then
    produce_video="${1#*=}"  # Get the value after '='
else
    echo "Error: Argument must be in the format 'produce_video=true' or 'produce_video=false'."
    exit 1
fi
cd ../../drive/model_training/data_utils || { echo "Directory not found"; exit 1; }


# Activate the Pyenv environment named 'data_analysis'
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init --path)"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"


if ! pyenv versions | grep -q "data_analysis"; then
    echo "Virtual environment 'data_analysis' does not exist."
    exit 1
fi

pyenv activate data_analysis


python3 update_drive_inventory.py --produce_video=$produce_video