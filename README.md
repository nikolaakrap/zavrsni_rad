# Robot tracking of human anatomy using a medical stereovision system
The aim of this paper is to integrate the collaborative robotic arm UR5e and the Polaris Vega
ST stereovision optical tracking system into the ROS2 operating system, establish their
connection, and prepare them for tracking a body part during a medical procedure. The study will explore how a collaborative robotic arm, in combination with such a system, can
precisely track anatomical structures such as the tibia and femur during knee surgery. During
surgery on these bones, the knee is never fully immobilized, so it is necessary to compensate
for all movements, even those that are invisible to the human eye

## Prerequisites
To get this project running, you'll need the following software and dependencies installed on your machine:

### 1. **Ubuntu 22.04**
This project is designed to run on **Ubuntu 22.04**.

### 2. **ROS 2 Humble**
This project is built on **ROS 2 Humble**. If you haven't installed ROS 2 Humble yet, follow the official installation guide:

- [ROS 2 Humble Installation Guide for Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Once ROS 2 is installed, remember to source the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### 3. **Required ROS 2 Packages**

The project relies on the following ROS 2 packages:

#### **Universal Robots ROS 2 Driver**
This package enables communication between ROS 2 and the Universal Robots robotic arms, such as the UR5e.

Install it via the following command:
```bash
sudo apt install ros-humble-ur-robot-driver
```

Alternatively, you can clone the package directly from GitHub if you're building from source:
```bash
cd ~/ros2_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
```

#### **SciKit-Surgery NDI Tracker**
This package allows communication with the Polaris Vega ST stereovision optical tracking system for precise tracking of anatomical structures.

Clone it into your workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/SciKit-Surgery/scikit-surgerynditracker.git
```

### 4. **Additional Dependencies**

#### **ROS Dependencies (via rosdep):**
Before building the workspace, ensure all the dependencies are installed:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5. **Development Tools**

#### **colcon**
A build tool used in ROS 2 workspaces. Install it with:
```bash
sudo apt install python3-colcon-common-extensions
```


## Running the Project

### 1. **Clone the Repository**

To start, clone this repository into your workspace directory:
```bash
cd ~/ros2_ws/src
git clone https://github.com/nikolaakrap/zavrsni_rad.git
```

### 2. **Install Dependencies**

Before building the workspace, you need to install all the necessary dependencies using rosdep:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. **Build the Workspace**

Now, build the workspace using colcon:
```bash
colcon build --symlink-install
```

### 4. **Source the Workspace**

After the build is complete, source the workspace to set up the ROS 2 environment:
```bash
source install/setup.bash
```

### 5. **Launch the project**

To start the tracking run the following command:
```bash
ros2 launch kamera_tf2_py kamera_tf2.launch.py
```

## Acknowledgements

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [SciKit-Surgery NDI Tracker](https://github.com/SciKit-Surgery/scikit-surgerynditracker)
