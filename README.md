# MAPF RoboSim – Setup and Run Guide

This guide will walk you through setting up and running the MAPF Robot Simulator — a multi-agent pathfinding simulator built with Gazebo, ROS 2, and a Flask-based web interface.

---

## 🛠️ Prerequisites

- **Operating System**: Ubuntu 24.04 (native or via WSL2)
- **Repository Location**: Clone the repository into your home directory:

  ```bash
  git clone https://github.com/noranh121/MAPF_RoboSim.git ~/MAPF_RoboSim
  ```

---

## 📦 Installation Steps

### 1. Install Gazebo Harmonic

Gazebo Harmonic is the latest Gazebo simulation distribution.  
👉 [Official Installation Guide](https://gazebosim.org/docs/harmonic/install_ubuntu/)

---

### 2. Install ROS 2 Jazzy (Recommended)

ROS 2 Jazzy is recommended for compatibility with Gazebo Harmonic.  
👉 [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

> **Note**: If you use a different ROS 2 distribution, replace `jazzy` in all terminal commands accordingly.

---

### 3. Install TurtleBot3

Install via Debian packages:

```bash
sudo apt install ros-jazzy-turtlebot3*
```

👉 [TurtleBot3 ROS 2 Guide](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)

---

### 4. Install ROS 2 & Python Dependencies

#### ROS 2 Tools:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-tf-transformations
```

#### Python and Pip:

```bash
sudo apt install python3-pip
```

> If you encounter issues installing with `pip`, try using a virtual environment (Recommended), or add the `--break-system-packages` flag (Use this flag with care):
>
> ```bash
> pip install <package> --break-system-packages
> ```

#### Required Python Packages:

```bash
pip install flask
pip install sortedcollections
pip install heapdict
pip install pygame
pip install loguru
```

---

## 🚀 Running the Simulator

1. Open a new terminal (Ubuntu 24.04 or WSL).
2. Navigate to the ROS 2 workspace:

    ```bash
    cd ~/MAPF_RoboSim/ros2_ws
    ```

3. Launch the Flask web app:

    ```bash
    python3 app/app.py
    ```

4. Open your browser and navigate to the address shown in the terminal (usually `http://127.0.0.1:5000`).

---

## 🧪 Quick Test

The simulator comes with default files ready to run:

- **Algorithm**: `lacam.py`
- **Map**: `benchmark.txt`
- **Scenario**: `scenario_test.txt`

To test:

1. Select the files in the web UI.
2. Click **Simulate**.
3. Gazebo should launch and run the simulation.

---

## 💾 Exporting Simulation Results

After the algorithm compeletion and the robots' paths are visualized via the Pygame window, you can download the results directly from the web interface.

Click the **"Download Results"** button in the UI to export data related to the current simulation. This may include metrics such as agent paths, execution time, and algorithm-specific statistics.

---

## ⚠️ Important Notes

- To **stop the simulation**, **use the "Stop Simulation" button** in the web UI.
  - ❌ Do **not** close the Gazebo window manually — it may leave processes running or crash the app.
- If commands fail, ensure your ROS environment is sourced:

    ```bash
    source /opt/ros/jazzy/setup.bash
    ```

    > Replace `setup.bash` with `setup.sh` or `setup.zsh` if using other shells.

---

## ✅ Compatibility Notes

The simulator is flexible and may work with different combinations of Ubuntu, Gazebo, and ROS 2 — as long as they are compatible.  
👉 [Gazebo Compatibility Chart](https://gazebosim.org/docs/latest/getstarted/)
