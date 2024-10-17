#!/bin/bash

# Check if a folder path argument is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path_to_folder>"
    exit 1
fi

# Assign the input argument to a variable
INPUT_FOLDER="$1"

# Check if the input path is a directory
if [ ! -d "$INPUT_FOLDER" ]; then
    echo "Error: '$INPUT_FOLDER' is not a valid directory."
    exit 1
fi

# Check if the extract_mapping script exists in the current directory
if [ ! -f "./extract_mapping.sh" ]; then
    echo "Error: 'extract_mapping' script not found in the current directory."
    exit 1
fi

# Loop through each subdirectory in the provided folder
for dir in "$INPUT_FOLDER"/*/; do
    # Check if it's a directory
    if [ -d "$dir" ]; then
        # Extract the full path
        FULL_PATH="$(realpath "$dir")"
        
        # Call the extract_mapping script with the directory path as an argument
        echo "Calling extract_mapping with path: $FULL_PATH"
        ./extract_mapping.sh "$FULL_PATH"
    fi
done
