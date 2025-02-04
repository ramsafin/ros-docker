# ROS Noetic with Docker

This project provides a setup for using ROS Noetic in a Docker container, making it easier to develop and test ROS packages within an isolated environment.

## Prerequisites

Before you begin, make sure you have the following installed:

- **Docker**: [Install Docker](https://docs.docker.com/get-docker/) for your platform.
- **Git**: [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) if you don't have it.
- **NVIDIA Container Toolkit**: [Install NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit?tab=readme-ov-file) if you have an NVIDIA GPU.

## Setting Up the Project

### 1. Clone the Project

If you haven't already, clone this repository to your local machine:

```bash
git clone https://github.com/ramsafin/ros-noetic-docker.git
cd ros-noetic-docker
```

### 2. Build the Docker Image

You can build the Docker image using the provided Makefile:
```bash
make build
```

This runs:
```bash
docker build -t ros-noetic -f docker/Dockerfile .
```

- `-t ros-noetic`: tags the image with the name `ros-noetic`
- `-f docker/Dockerfile`: specifies the path to the `Dockerfile`
- `.`: sets the build context to the current project directory

### 3. Running the Docker Container

To start a container with GUI support and mount your ROS workspace, use:
```bash
make run
```

### 4. Building the ROS Workspace

Once inside the container, navigate to your ROS workspace and build it:

```bash
cd /catkin_ws && catkin_make
```

Alternatively, using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/):
```bash
cd /catkin_ws && catkin build
```

This will generate the `build/`, `devel/`, and `install/` directories in the `catkin_ws` directory.

### 5. Running ROS Nodes

After building your workspace, you can start working with ROS nodes. For example, you can run the ROS master:

```bash
roscore
```

Do not forget to source your ROS environment and workspace:
```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
```

### 6. Stopping the Container

When you're finished, you can stop the container by typing `exit` or pressing `Ctrl+D`. Since the container was started with `--rm`, it will be automatically removed when stopped.

To manually stop the container, run:
```bash
make stop
```

### 7. Troubleshooting

A. [GPU selection in WSgl (WSL2)](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)
B. [Containerizing GUI applications with WSLg](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)

## Contributing

If you'd like to contribute to this project, feel free to fork the repository and submit a pull request. You can also open an issue if you encounter any bugs or have suggestions for improvement.

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
