#!/bin/bash

# Get the directory of the script
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Navigate one directory up to get the base_directory
base_directory="$(dirname "$script_dir")"

# Change the current working directory to the base_directory
cd "$base_directory" || exit

# Remove the "Logs" and "build" directories using sudo
sudo rm -rf "Logs" "build"

# Check if the removal was successful
if [ $? -eq 0 ]; then
    echo "Removed 'Logs' and 'build' directories."
else
    echo "Failed to remove 'Logs' and 'build' directories."
fi
