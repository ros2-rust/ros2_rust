#.. redirect-from::
#
#    Deploying-ROS2-on-IBM-Cloud
#    Tutorials/Deploying-ROS-2-on-IBM-Cloud

Development Container for ROS2 Rust
====================================

This repository includes a pre-configured development container setup, designed to facilitate working on ROS2 Rust projects. The setup is managed using Visual Studio Code's DevContainers feature.

.. contents:: Table of Contents
   :depth: 3
   :local:

File Description
----------------

The provided ``devcontainer.json`` file specifies the configuration for running a containerized development environment with the following features:

- **Image Configuration**: Uses the Docker image ros2_rust_dev:latest.
- **Build Settings**:
    - Context: Specifies the root directory of the project (``..``).
    - Dockerfile: Uses the Dockerfile located at ``../docker/Dockerfile``.
    - Build Args: Includes runtime environment variables such as ``XDG_RUNTIME_DIR``.
- **Workspace**:
    - Mounts the folder ``/home/cuser/workspace`` as the development workspace.
- **Container Runtime**:
    - Enables ``--network=host`` for network sharing.
    - Grants privileged access using ``--privileged``.
- **Environment Variables**:
    - ``DISPLAY`` and ``TERM`` are forwarded from the host to the container to support GUI applications and terminal colors.
- **User Settings**:
    - The development container runs as the user ``cuser``.
    - Configures the shell to use ``/bin/bash`` for a familiar and customizable command-line experience.

Setting Up and Running the DevContainer in VS Code
--------------------------------------------------

To use this development container in Visual Studio Code, follow these steps:

Prerequisites
~~~~~~~~~~~~~

1. **Install VS Code**: Download and install `Visual Studio Code <https://code.visualstudio.com/>`__.

2. **Install Required Extensions:**
    - `Remote Containers <https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers>`__
3. **Install Docker:** Ensure Docker is installed and running on your system. You can download Docker from `here <https://www.docker.com/>`__.


Steps to Launch the DevContainer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Clone the Repository**: Clone your project repository to your local machine.

.. code-block:: bash

   $ git clone git@github.com:ros2-rust/ros2_rust.git
   $ cd ros2_rust



2. **Open in VS Code**: Open the repository folder in VS Code.

.. code-block:: bash

   $ code .

3. **Open the Command Palette**: Press ``Ctrl+Shift+P`` (Windows/Linux) or ``Cmd+Shift+P`` (macOS) to open the Command Palette.
4. **Reopen in Container**: Search for and select **"Dev Containers: Reopen in Container"**. VS Code will build and start the development container based on the configuration in ``devcontainer.json``.
5. **Start Coding**: Once the container is running, your workspace is ready for development. You can open terminals, run commands, and debug your ROS2 Rust projects seamlessly within the containerized environment.

Notes
~~~~~

- **Container Network**: The ``--network=host`` option is enabled to allow network sharing between the host and container.
- **Privilege**: The ``--privileged`` flag is enabled, which may be required for specific ROS2 features but should be used cautiously.
- If you encounter permission issues with X11, ensure the host system is configured to allow access in each terminal:

.. code-block:: bash

   $ xhost local:root
   $ code &