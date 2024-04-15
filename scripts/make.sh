#!/bin/bash

# Initialize a flag to determine if we should run cmake and make
run_cmake_and_make=true

# Check for the --test flag
while [[ $# -gt 0 ]]; do
    case "$1" in
        --test)
            run_cmake_and_make=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Get the directory of the script
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Navigate one directory up to get the base_directory
base_directory="$(dirname "$script_dir")"

# Change the current working directory to the base_directory
cd "$base_directory" || exit

# Create a "build" subdirectory in the base_directory
build_directory="build"

# Check if the "build" directory already exists
if [ ! -d "$build_directory" ]; then
    # If it doesn't exist, create it
    mkdir "$build_directory"
    echo "Created 'build' directory in $base_directory"
fi

# Change to the "build" directory if we are not testing
if [ "$run_cmake_and_make" = true ]; then
    cd "$build_directory" || exit

    # Run cmake and make
    cmake ..
    make -j4
else
    echo "Skipping cmake and make"
fi

# Change the current working directory to the base_directory
cd "$base_directory" || exit
