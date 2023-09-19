#!/bin/bash

# Get the directory containing this script
script_directory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Get the parent directory (one level up)
base_directory="$(dirname "$script_directory")"

# Define the config directory
config_directory="$base_directory/config"

# Check if config_directory exists and create it if it doesn't
if [ ! -d "$config_directory" ]; then
  mkdir -p "$config_directory"
  echo "Created config directory: $config_directory"
else
  echo "Config directory already exists: $config_directory"
fi

# List all files in config_directory
config_files=("$config_directory"/*)

# Search for files with the same name in subdirectories of base_directory
for config_file in "${config_files[@]}"; do
    config_filename=$(basename "$config_file")
    found_files=$(find "$base_directory" -type f -not -path "$config_directory/*" -name "$config_filename")
    for found_file in $found_files; do
        relative_path=$(realpath --relative-to="$(dirname "$found_file")" "$config_file")
        echo "Replacing $found_file with a symlink to $config_file"
        rm "$found_file"  # Remove the existing file
        ln -s "$relative_path" "$found_file"  # Create a symlink with the relative path
    done
done
