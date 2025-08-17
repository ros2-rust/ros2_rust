#!/usr/bin/env python3
import ament_index_python
import os
import subprocess
import sys

def main():
    if len(sys.argv) != 4:
        print("Usage: generate_bindings.py <header_file> <ros_distribution> <output_directory>")
        sys.exit(1)

    header_file = sys.argv[1]
    ros_distribution = sys.argv[2]
    output_directory = sys.argv[3]
    bindgen_command = []
    bindgen_command.append('bindgen')
    bindgen_command.append(header_file)
    bindgen_command.append('-o')
    bindgen_command.append(f'{output_directory}/rcl_bindings_generated_{ros_distribution}.rs')
    bindgen_command.append('--rust-edition')
    bindgen_command.append('2021')
    bindgen_command.append('--rust-target')
    bindgen_command.append('1.75')
    bindgen_command.append('--no-derive-copy')
    bindgen_command.append('--allowlist-type')
    bindgen_command.append('rcl_.*')
    bindgen_command.append('--allowlist-type')
    bindgen_command.append('rmw_.*')
    bindgen_command.append('--allowlist-type')
    bindgen_command.append('rcutils_.*')
    bindgen_command.append('--allowlist-type')
    bindgen_command.append('rosidl_.*')
    bindgen_command.append('--allowlist-function')
    bindgen_command.append('rcl_.*')
    bindgen_command.append('--allowlist-function')
    bindgen_command.append('rmw_.*')
    bindgen_command.append('--allowlist-function')
    bindgen_command.append('rcutils_.*')
    bindgen_command.append('--allowlist-function')
    bindgen_command.append('rosidl_.*')
    bindgen_command.append('--allowlist-var')
    bindgen_command.append('rcl_.*')
    bindgen_command.append('--allowlist-var')
    bindgen_command.append('rmw_.*')
    bindgen_command.append('--allowlist-var')
    bindgen_command.append('rcutils_.*')
    bindgen_command.append('--allowlist-var')
    bindgen_command.append('rosidl_.*')
    bindgen_command.append('--no-layout-tests')
    bindgen_command.append('--default-enum-style')
    bindgen_command.append('rust')
    bindgen_command.append('--')
    for package, prefix in ament_index_python.get_packages_with_prefixes().items():
        package_include_dir = os.path.join(prefix, 'include', package)
        if os.path.isdir(package_include_dir):
            bindgen_command.append('-isystem')
            bindgen_command.append(package_include_dir)
    subprocess.run(bindgen_command)

if __name__ == "__main__":
    main()
