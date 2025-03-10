# Building `ros2_rust` packages

This is a more detailed guide on how to build ROS 2 packages written in Rust that expands on the minimal steps in the README.

In this guide, the Humble distribution of ROS 2 is used, but newer distributions can be used by simply replacing 'humble' with the distribution name everywhere.


## Choosing a workspace directory and cloning `ros2_rust`

A "workspace directory", or just "workspace", is simply a directory of your choosing that contains your `ros2_rust` checkout and potentially other ROS 2 packages. It will also usually be your working directory for building. There is only one limitation: It must not contain the ROS 2 installation, so it can't be `/`, for instance. Note that this has **nothing** to do with a [`cargo` workspace](https://doc.rust-lang.org/book/ch14-03-cargo-workspaces.html).

If you don't have a workspace directory already, simply create an empty directory in a convenient location.

Next, clone `ros2_rust` into it. The rest of this article will assume that you have cloned it into a subdirectory named `src` like this:

```shell
# Make sure to run this in the workspace directory
mkdir src
git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
```


## Environment setup

Building `rclrs` requires a standard [ROS 2 installation](https://docs.ros.org/en/humble/Installation.html), and a few extensions.
These extensions are: `colcon-cargo`, `colcon-ros-cargo`, `cargo-ament-build`. The first two are `colcon` plugins, and the third is a `cargo` plugin.

The `libclang` library is also required for automatically generating FFI bindings with `bindgen`. See the [`bindgen` docs](https://rust-lang.github.io/rust-bindgen/requirements.html) on how to install it. As a side note, on Ubuntu the `clang` package is not required, only the `libclang-dev` package.

The `python3-vcstool` package is used in [importing auxiliary repositories](#importing-repositories). It can also be installed through `pip` instead of `apt`.

You can either install these dependencies on your computer, or use the [premade Docker image](#using-the-docker-image).


### Option 1: Installing the dependencies

The exact steps may differ between platforms, but as an example, here is how you would install the dependencies on Ubuntu:

<!--- These steps should be kept in sync with README.md --->
```shell
# Install Rust, e.g. as described in https://rustup.rs/
# Install ROS 2 as described in https://docs.ros.org/en/humble/Installation.html
# Assuming you installed the minimal version of ROS 2, you need these additional packages:
sudo apt install -y git libclang-dev python3-pip python3-vcstool # libclang-dev is required by bindgen
# Install these plugins for cargo and colcon:
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

### Option 2: Using the Docker image

Build the Docker image with

```shell
# Make sure to run this in the workspace directory
# ROS_DISTRO can be humble|rolling
docker build -f src/ros2_rust/Dockerfile --build-arg "ROS_DISTRO=humble" -t ros2_rust_dev .
```

and then run it with

```shell
# Make sure to run this in the workspace directory
docker run --rm -it --volume $(pwd):/workspace ros2_rust_dev /bin/bash
```

As you can see, this maps the workspace directory to `/workspace` inside the container. So, if you're using Docker, that is now the "workspace directory".


### Importing repositories

`ros2_rust` also expects a few other repositories to be in the workspace. They can be imported from a `repos` file contained in `ros2_rust`, like this:

```shell
# Make sure to run this in the workspace directory
vcs import src < src/ros2_rust/ros2_rust_humble.repos
```

This uses the [`vcs` tool](https://github.com/dirk-thomas/vcstool), which is preinstalled in the Docker image.

The new repositories are now in `src/ros2`.
The list of needed repositories should very rarely change, so you typically only need to do this once.


### Sourcing the ROS 2 installation

Before building, the ROS 2 installation, and ideally _only_ the ROS 2 installation, should be sourced.
Sourcing an installation populates a few environment variables such as `$AMENT_PREFIX_PATH`, so if you are not sure, you can check that the `$AMENT_PREFIX_PATH` environment variable contains the ROS 2 installation, e.g. `/opt/ros/humble`.

In the pre-made Docker image, sourcing is already done for you. Otherwise, if `$AMENT_PREFIX_PATH` is empty, you need to source the ROS 2 installation yourself:

```shell
# You can also do `source /opt/ros/humble/setup.bash` in bash
. /opt/ros/humble/setup.sh
````

If `$AMENT_PREFIX_PATH` contains undesired paths, exit and reopen your shell. If the problem persists, it's probably because of a sourcing command in your `~/.bashrc` or similar.

It's convenient to install `tmux`, especially in Docker, and open separate windows or panes for building and running.


### Checking your setup

To verify that you've correctly installed dependencies and sourced your ROS 2 installation, you should be able to run

```shell
colcon list
```

without errors and see a line like this in the output:

```
rclrs   src/ros2_rust/rclrs   (ament_cargo)
```

The build type `ament_cargo` means that the `colcon-ros-cargo` plugin works as expected.


## Building with `colcon`

Now that everything is set up, you can build with `colcon`. The basic command to build a package and its dependencies is

```shell
# Make sure to run this in the workspace directory
colcon build --packages-up-to $YOUR_PACKAGE
```

where `$YOUR_PACKAGE` could be e.g. `examples_rclrs_minimal_pub_sub`. See `colcon build --help` for a complete list of options.

It's normal to see a `Some selected packages are already built in one or more underlay workspace` warning. This is because the standard message definitions that are part of ROS 2 need to be regenerated in order to create Rust bindings. If and when `ros2_rust` becomes an officially supported client library, this issue will disappear.

In addition to the standard `build`, `install` and `log` directories, `colcon` will have created a `.cargo/config.toml` file.


### Tips and limitations

Don't start two build processes involving Rust packages in parallel; they might overwrite each other's `.cargo/config.toml`.

A clean build will always be much slower than an incremental rebuild.

`colcon test` is not currently implemented for Rust packages.

The plugin to build `cargo` projects with `colcon` currently has the issue that both `cargo` and `colcon` will [rebuild all dependencies for each package](https://github.com/colcon/colcon-ros-cargo/). This makes building large projects with `colcon` less efficient, but if this is an issue, you can build with `cargo` instead.


## Building with `cargo`

Rust packages for ROS 2 can also be built with pure `cargo`, and still integrate with ROS 2 tools like `ros2 run`.


## Integration with ROS 2 tools

How can a binary created in Rust be made available to `ros2 run`, `ros2 launch` etc.? And how can other ROS 2 packages use a `cdylib` created in Rust? For that to work, the correct files must be placed at the correct location in the install directory, see [REP 122](https://www.ros.org/reps/rep-0122.html).

It's not necessary to learn about which marker file goes where. The functionality to properly set up the install directory was extracted from `colcon-ros-cargo` into a `cargo` plugin, so that `colcon` is not required: [`cargo-ament-build`](https://github.com/ros2-rust/cargo-ament-build).

Simply use `cargo ament-build --install-base <path to install dir>` as a drop-in replacement for `cargo build`. After building, run `. install/setup.sh` and you're good to run your executable with `ros2 run`.


## Troubleshooting

If something goes very wrong and you want to start fresh, make sure to delete all `install*`, `build*`, `target`, and `.cargo` directories. Possibly update the Docker container if it is out of date.


### Package identification

If you forget to source the ROS 2 installation, you'll get this error:

> ERROR:colcon.colcon_core.package_identification:Exception in package identification extension 'python_setup_py'

Source the ROS 2 installation and try again.


### Failed to resolve patches

If you've deleted your `install` directory but not your `.cargo` directory, you'll get this error:

> error: failed to resolve patches for `https://github.com/rust-lang/crates.io-index`

Delete the `.cargo` directory, and rebuild.


### Can not find library at runtime

If you `cargo run` a package that depends on a custom message, or some other dynamic library outside of the ROS 2 distribution, you will see an error from the dynamic linker such as:

> target/debug/message_demo: error while loading shared libraries: librclrs_example_msgs__rosidl_typesupport_c.so: cannot open shared object file: No such file or directory

To help your executable find the library, one possibility is to source the install directory which contains the message (or other library) package. This will add the required entry to the `$LD_LIBRARY_PATH` variable.
