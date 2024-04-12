project_dirs=(
  "./rclrs"
  "./rosidl_runtime_rs"
  "./examples/message_demo"
  "./examples/minimal_client_service"
  "./examples/minimal_pub_sub"
  "./examples/your_package_name/"
)

# Loop through each subdirectory
for project_dir in "${project_dirs[@]}"; do
  # Check if the directory exists
    cd "$project_dir"

    # Run cargo fmt
    cargo-fmt
    cargo-clippy

    # Change back to the root directory
    cd "./"

done

echo "Formatting completed!"
