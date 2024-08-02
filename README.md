# ros_uwb_dwm1001

## Project Description

**ros_uwb_dwm1001** is a ROS Noetic package designed for seamless integration with the [Qorvo DWM1001](https://www.qorvo.com/products/p/DWM1001-DEV) ultra-wideband (UWB) modules, providing enhanced real-time location tracking and positioning capabilities. This package is built upon existing open-source projects from the Human and Intelligent Vehicle Ensembles (HIVE) Lab and extends their functionality to support ROS 1 Noetic environments.

## Key Features

- **UWB Positioning with ROS Integration**: Utilizes Qorvo DWM1001 modules to publish UWB positioning data as `PointStamped` messages over the ROS network, enabling easy access to precise location data in ROS-compatible applications.

- **Transform Calculations**: Converts UWB positions into map-space odometry data, facilitating integration with other robotic systems and navigation algorithms within the ROS framework.

- **Easy Deployment and Use**: Offers straightforward installation and configuration steps, making it accessible to users with varying levels of experience in robotics and ROS.

## Modifications and Enhancements

- **ROS Noetic Compatibility**: Adapted the original ROS 2 package to be compatible with ROS 1 Noetic, providing a bridge for users who have not yet transitioned to ROS 2.

- **Customizable Topics**: Allows for custom topic names and message types, ensuring flexibility in how data is published and consumed within your robotic applications.

- **Enhanced Data Handling**: Improved error handling and data parsing to ensure robustness and reliability in dynamic and noisy environments.

## Usage Scenarios

- **Robotics Localization**: Integrate with mobile robots to enhance their localization and navigation systems using precise UWB positioning data.

- **Industrial Automation**: Deploy in manufacturing and warehouse environments to track assets, monitor worker locations, and improve workflow efficiency.

- **Research and Development**: Use as a platform for developing and testing new algorithms and applications in robotics and IoT.

## Getting Started

To get started with the `ros_uwb_dwm1001` package, follow the steps below to clone the repository, build the package, and configure your ROS environment.

### Prerequisites

- **ROS Noetic**: Ensure that you have ROS Noetic installed on your system. You can find installation instructions for ROS Noetic [here](http://wiki.ros.org/noetic/Installation).

- **Catkin Workspace**: You should have a catkin workspace set up. If not, you can create one using the instructions below.

### Step-by-Step Installation

1. **Set Up Your Catkin Workspace**

   If you do not already have a catkin workspace, you can create one with the following commands:

   ```bash
   # Create a directory for your workspace
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src

   # Initialize the workspace
   catkin_init_workspace
   ```

 2. **Clone the Repository**

    Navigate to the `src` directory of your catkin workspace and clone the repository:
	
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/mohd-nazrin/ros_uwb_dwm1001.git
    ```
    
3. **Build the Package**
    After cloning the repository, build the package using catkin_make:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

   This command will compile the package and set up the necessary dependencies.
4. **Source Your Workspace**
   Before using the package, you need to source your catkin workspace to make the new package available:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```
   
   Add this line to your .bashrc file to ensure that your workspace is automatically sourced every time you open a new terminal:
   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
   
5. **Configure and Run the Nodes**
   Before launch, check scripts/dwm1001c.py file to ensure correct serial port:
   ```bash
   serial_handle = Serial('/dev/ttyACM0', 115200, timeout=15)  # Update port as necessary
   ```
   Launch the provided ROS launch file to start the package and its nodes. This will automatically start the UWB positioning and transformation nodes:
   ```bash
   roslaunch ros_uwb_dwm1001 uwb_odometry.launch
   ```
   This command will start the nodes necessary for publishing UWB positioning data and performing coordinate transformations.
7. **Verify the Installation**
   Use the following commands to verify that the nodes are running and the topics are being published:
   ```bash
   # List all active topics
   rostopic list
   
   # Echo the UWB position topic to verify data output
   rostopic echo /uwb_pos
   
   # Echo the transformed odometry data
   rostopic echo /odometry/ips
   ```

### Acknowledgments

This project is based on code from the Human and Intelligent Vehicle Ensembles (HIVE) Lab, originally licensed under the Apache License 2.0 and the MIT License. For more details, please see the LICENSE and NOTICE files in this repository.

   
