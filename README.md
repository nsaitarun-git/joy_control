# Joy Control

**Joy Control** is a ROS2 package designed to integrate joystick inputs into robotic control systems. It facilitates the mapping of joystick commands to various robot functionalities, enabling intuitive and responsive manual control.

---

## Features

- **Joystick Integration**: Seamlessly connects compatible joysticks to the ROS ecosystem.
- **Customizable Mappings**: Allows users to define specific joystick-to-robot command mappings.
- **Real-time Control**: Enables immediate response to joystick inputs for precise maneuvering.

---

## Requirements

- **ROS 2 Humble**: The latest version of the Robot Operating System.
- **Gamepad**: For this project I used a PS4 controller to move a differential drive robot.

---

## Getting Started

### Cloning the package
Clone the package into a workspace and build it. Create a workspace with the following commands if you haven't done so already.

```bash
#Create workspace and source folder
mkdir joy_control_ws
cd joy_control_ws
mkdir src
cd src

# Clone the package
git clone https://github.com/nsaitarun-git/joy_control.git

# Build the workspace
cd ..
colcon build --symlink-install
```
---

### Usage
Within the ```joy_control_ws``` folder, source the installation and run the launch file with your gamepad connected to the PC via USB or Bluetooth. 
```bash
# Source the installation
source install/setup.bash

# Run the launch file
ros2 launch joy_control joystick.launch.py 
```
To view the data being published to ```/cmd_vel``` topic, use the following command.
```bash
# See the data published from gamepad
ros2 topic echo /cmd_vel
```
---

# License
This project is licensed under the MIT License. See the [LICENSE](https://github.com/nsaitarun-git/joy_control/blob/main/LICENSE) file for details.
