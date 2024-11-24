#.. redirect-from::
#
#    Deploying-ROS2-on-IBM-Cloud
#    Tutorials/Deploying-ROS-2-on-IBM-Cloud

ROS2 Rust Docker Environment
=========================================================

This folder contains scripts to build, start, and interact with a Docker container for a ROS2 Rust environment.

.. contents:: Table of Contents
   :depth: 3
   :local:

Prerequisites
-------------

Ensure you have the following installed on your system:

-  `Docker <https://docs.docker.com/engine/install/>`__
- Permissions to run Docker commands (sudo may be required on some systems). 



Files in the Folder
-------------------

- `build.sh <./build.sh>`_: Builds the Docker image.
- `start.sh <./start.sh>`_: Starts the Docker container.
- `shell.sh <./shell.sh>`_: Accesses the running container


Steps to Use
------------

1. Build the Docker Image
~~~~~~~~~~~~~~~~~~~~~~~~~

Run the ``build.sh`` script to build the Docker image:

.. code-block:: bash

   $ ./build.sh

This script:

- Builds a Docker image named ``ros2_rust:latest``.
- Uses ``BUILDKIT`` for efficient caching and builds.


2. Start the Docker Container
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Run the ``start.sh`` script to start the container:

.. code-block:: bash

   $ ./start.sh

This script:

- Checks if a container based on the ros2_rust image exists and removes it if necessary.
- Starts a new container named ros2_rust with the following settings:
      - **Interactive mode** (``-it``).
      - **User ID Mapping**: Maps the current user (ID ``1000``) to the container user cuser. 
      - **Privileges**: Grants ``sudo`` group membership and system capabilities (e.g., sys_nice).
      - **Networking**: Uses the host's network stack.
      - **X11 Forwarding**: Allows graphical applications to be displayed on the host system.


3. Access the Running Container
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Run the ``shell.sh`` script to open an interactive shell in the running container:

.. code-block:: bash

   $ ./shell.sh

This script:

- Opens a Bash shell as the user ``cuser``.
- Sets the working directory to ``/home/cuser/workspace``.


4. Install ros2 rust packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Follow the `README.md <../README.md>`_ file to install the overall packages needed to start working with ROS2 and Rust.


Notes
-----
- If you encounter permission issues with X11, ensure the host system is configured to allow access in each terminal:

.. code-block:: bash

   $ xhost local:root