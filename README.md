# ROS Noetic with Docker

This project provides a ready-to-use Docker container for ROS Noetic, optimized for development and testing. It simplifies the process of working with ROS Noetic in an isolated, reproducible environment that can run on both Windows (via WSL) and Linux. The setup includes GUI support for ROS development in WSLg and native Linux systems, along with easy integration with your existing ROS workspace.

## Prerequisites

Before you begin, make sure you have the following installed:

- **Docker**: [install Docker](https://docs.docker.com/get-docker/) for your platform.
- **NVIDIA Container Toolkit**: [install NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit?tab=readme-ov-file) if you have an NVIDIA GPU.
- Windows Subsystem for Linux (WSL): [install WSL](https://learn.microsoft.com/ru-ru/windows/wsl/install) if you want to run Docker containers in Windows.

Ensure you have WSLg installed, as it enables GUI support for Docker containers in Windows. If you're unsure about this, refer to the [WSLg Installation guide](https://github.com/microsoft/wslg).

## Setting Up the Project

### 1. Clone the Project

If you haven't already, clone this repository to your local machine:

```bash
git clone https://github.com/ramsafin/ros-noetic-docker.git
cd ros-noetic-docker
```

### 2. Build the Docker Image

There are two build options available depending on your environment: WSL and Linux.

Build the Docker image optimized for WSL:
```bash
make build_wsl
```

Build the Docker image optimized for Linux:
```bash
make build_linux
```

The [Makefile](Makefile) handles the setup for different environments, adjusting the Dockerfile build arguments accordingly.

### 3. Running the Docker Container

To start a container with GUI support and **mount your ROS workspace**, use:
```bash
make run_wsl
```
or
```bash
make run_linux
```

This will automatically handle display forwarding if you are using WSLg. For non-WSLg setups, you may need to configure access to the X server (e.g., with `xhost` in non-WSL2 environments).

### 4. Building the ROS Workspace

By default your ROS workspace inside the container is `ROS_WORKSPACE=/home/ros/catkin_ws`.

Once inside the container, navigate to your ROS workspace and build it:

```bash
cd $ROS_WORKSPACE && catkin_make
```

Alternatively, using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/):
```bash
cd $ROS_WORKSPACE && catkin build
```

This will generate the `build/`, `devel/`, and `install/` directories in the `catkin_ws` directory.

If you have uninstalled dependencies, use:
```bash
rosdep install --from-paths $ROS_WORKSPACE/src --ignore-src -y
```

### 5. Running ROS Nodes

After building your workspace, you can start working with ROS nodes. For example, you can run the ROS master:

```bash
roscore
```

Do not forget to source your ROS environment and workspace:
```bash
source /opt/ros/noetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash
```

### 6. Stopping the Container

When you're finished, you can stop the container by typing `exit` or pressing `Ctrl+D`. Since the container was started with `--rm`, it will be automatically removed when stopped.

To manually stop the container, run:
```bash
make stop
```

## Troubleshooting

### How to select GPU in WSLg?

The NVIDIA GPU can be selected in WSL by setting `MESA_D3D12_DEFAULT_ADAPTER_NAME`:
```bash
echo "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" >> ~/.bashrc
source ~/.bashrc
```
Follow the instructions in the [GPU selection in WSLg](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg) guide.

### Containerizing GUI apps with WSLg

Follow the instructions in the [wslg/containers](https://github.com/microsoft/wslg/blob/861d029e97bc99e68050f86c956803b42e8756da/samples/container/Containers.md) guide.

### Docker container cannot connect to X server

Ensure that the `xhost` command is used to allow Docker to connect to the X server:
```bash
xhost +SI:localuser:$(whoami)
```

Additionally, verify that your X server is running.

### How to check that NVIDIA is used for OpenGL rendering?

Check the output of the command:
```bash
glxinfo -B | grep Device
```

It should be your NVIDIA GPU.

## Auxiliary

1. For more detailed documentation on setting up and using ROS Noetic, visit the [official ROS documentation](http://wiki.ros.org/noetic).
2. For more information about WSLg, visit the [WSLg official repository](https://github.com/microsoft/wslg).

## Contributing

If you'd like to contribute to this project, feel free to fork the repository and submit a pull request. You can also open an issue if you encounter any bugs or have suggestions for improvement.

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
