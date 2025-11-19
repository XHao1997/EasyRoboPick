# EasyRoboPick – Dynamixel AX-12A Control (ROS 2)

This part of **EasyRoboPick** provides a ROS 2 interface for controlling a robot built from **Dynamixel AX-12A** servos.

It includes:

- A C++ node `read_write_node_ax12a` (run via `dynamixel_sdk_examples read_write_node`)
- Motion command interface in **radians**
- A service for reading **current joint positions** in radians

---

## 1. Requirements

- **ROS 2** (Humble or later)
- **Dynamixel SDK** (Protocol 1.0)
- Packages:
  - `dynamixel_sdk`
  - `dynamixel_sdk_custom_interfaces`
  - `dynamixel_sdk_examples`
- A working Dynamixel bus (e.g. USB2Dynamixel / U2D2) with AX-12A servos

Make sure your user has permission to access the serial device (e.g. `/dev/ttyUSB0`).

---

## 2. Clone the Repository (with Submodules)

Clone **EasyRoboPick** and initialize all submodules in one command:

```bash
git clone --recurse-submodules git@github.com:XHao1997/EasyRoboPick.git
cd EasyRoboPick
```
If you already cloned without --recurse-submodules, fix it with:
```bash
git submodule update --init --recursive
```
## Build Workspace
From the EasyRoboPick root:
```bash
colcon build
source install/setup.bash
```
## 4. Node Overview

The main node is:

- Node name: read_write_node_ax12a

- Executable: read_write_node in package dynamixel_sdk_examples

What it does:

- Reads parameters (serial port, baudrate, motor IDs, joint names)

- Opens the Dynamixel serial port

- Sets baudrate

- Enables torque on all configured IDs

Exposes:

    /move_joint topic (move all joints in radians, sync-write)

    /set_position topic (set a single joint in ticks)

    /set_multi_position topic (set multiple joints in ticks)

    /get_position service (read joint positions in radians)

## Start the AX-12A Node
#### Terminal 1 (from EasyRoboPick root):
    source install/setup.bash
    ros2 run dynamixel_sdk_examples read_write_node

## Move the Robot to Zero Position
#### Terminal 2 (same directory):
    source install/setup.bash
    ros2 topic pub -1 /move_joint \
    dynamixel_sdk_custom_interfaces/msg/MoveJoint \
    "{joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

## Read All Joint Positions via Service
#### Terminal 3:
    source install/setup.bash
    ros2 service call /get_position \
    dynamixel_sdk_custom_interfaces/srv/GetPosition \
    "{id: -1}"
