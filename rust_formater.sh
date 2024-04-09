#!/bin/bash

# Define the root directory containing the Cargo projects
root_dir=$(pwd)

# List of subdirectories containing Cargo projects
project_dirs=(
  "rclrs"
  "rosidl_runtime_rs"
  "examples/message_demo"
  "examples/minimal_client_service"
  "examples/minimal_pub_sub"
)

# Loop through each subdirectory
for project_dir in "${project_dirs[@]}"; do
  # Check if the directory exists
  if [ -d "$root_dir/$project_dir" ]; then
    # Change directory to the project folder
    cd "$root_dir/$project_dir"

    # Run cargo fmt
    cargo-fmt
    cargo-clippy

    # Change back to the root directory
    cd "$root_dir"
  else
    echo "Skipping $project_dir: Directory does not exist."
  fi
done

echo "Formatting completed!"
