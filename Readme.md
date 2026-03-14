# Autonomous Mobile Robots - UFSCar 🤖

This repository contains the projects and exercises developed for the **Autonomous Mobile Robots** course at the Federal University of São Carlos (UFSCar). The goal of this environment is to explore autonomous navigation, path planning, and perception for Unmanned Aerial Vehicles (UAVs).

## 🛠 Tech Stack

- **OS:** Ubuntu 24.04 (Noble Numbat)
- **Framework:** [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
- **Simulator:** [MRS UAV System (ROS 2)](https://github.com/ctu-mrs/mrs_uav_system/tree/ros2)
- **Environment:** Docker with VS Code Dev Containers

## 🚀 Getting Started

The environment is fully containerized to ensure reproducibility and to isolate dependencies from your host system.

### Prerequisites

1.  **Ubuntu 22.04+** (Host system)
2.  **Docker Engine** installed and configured.
3.  **VS Code** with the **Dev Containers** extension.
4.  **X11 Server** access (for GUI tools like Gazebo and RViz).

### Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/lucasfroes232/autonomous_mobile_robots.git](https://github.com/lucasfroes232/autonomous_mobile_robots.git)
    cd autonomous_mobile_robots
    ```

2.  **Grant GUI permissions:**
    On your host machine, run this command to allow the Docker container to open windows (RViz/Gazebo):
    ```bash
    xhost +local:root
    ```

3.  **Open in VS Code:**
    ```bash
    code .
    ```

4.  **Launch the Container:**
    - When prompted by VS Code, click **"Reopen in Container"**.
    - If the prompt doesn't appear, press `F1`, type `Dev Containers: Reopen in Container`, and press Enter.
    - *Note: The first build may take a few minutes as it downloads ROS 2 and the MRS UAV System.*

### Running a Simulation

Once inside the container terminal, you can launch a basic 3D Gazebo simulation:

```bash
cd /opt/ros/jazzy/share/mrs_uav_gazebo_simulation/tmux/mrs_one_drone
./start.sh
```

---
## 📂 Project Struture

* `.devcontainer/`:Definition of the Docker environment and VS Code extensions.
* `src/`: Custom ROS 2 packages and nodes. 

---
## Author

**Lucas Froes Belinassi**
Undergraduate student in Computer Engineering at **UFSCar (São Carlos)**
Researcher at **LARIS (Laboratory of Autonomous Robots and Intelligent Systems)**.
Research Interests: Path Planning, UAVs, SLAM, Mobile Robotics and Embedded Systems.