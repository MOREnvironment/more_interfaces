# ✉️ MORE Interfaces

`more_interfaces` is a ROS 2 interface package that extends the standard [`common_interfaces`](https://github.com/ros2/common_interfaces) and other generic ROS 2 interface packages with **maritime-specific** messages, services, and actions.

The goal is not to duplicate existing ROS 2 interfaces for poses, paths, routes, or maps, but to add the missing pieces needed for:

- AUV / ASV / USV operations
- Underwater navigation and acoustic sensing
- Maritime mission planning and execution
- Multi-vehicle cooperative missions in marine environments

## Table of contents

- [1. Repository structure](#1-repository-structure)
- [2. Prerequisites](#2-prerequisites)
  - [2.1. Create or reuse a ROS 2 workspace](#21-create-or-reuse-a-ros-2-workspace)
- [3. Clone the repository](#3-clone-the-repository)
- [4. Build the package](#4-build-the-package)
- [4. Use the package](#4-use-the-package)
  - [4.1. Check the interfaces](#41-check-the-interfaces)
  - [4.2. Using the messages in your own code](#42-using-the-messages-in-your-own-code)
- [5. Planned maritime-specific interfaces](#5-planned-maritime-specific-interfaces)
- [6. Existing ROS 2 interfaces](#6-existing-ros-2-interfaces)
- [📝 Credits](#-credits)

## 1. Repository structure

The repository follows the usual ROS 2 interface layout:

```text
more_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/        # Maritime-specific message definitions 
├── srv/        # Maritime-specific service definitions 
└── action/     # Maritime-specific action definitions 
└── ...
```

## 2. Prerequisites

- ROS 2 **Jazzy** installed (and working `ros2` CLI)
- `colcon` build tools installed:
  ```bash
  sudo apt update
  sudo apt install python3-colcon-common-extensions
  ```
- A ROS 2 workspace (e.g. `more_ws`) or any other worskpace


### 2.1. Create or reuse a ROS 2 workspace

If you already have a workspace (e.g. `~/more_ws`) or other worskpace, you can skip this step.

```bash
mkdir -p ~/more_ws/src
cd ~/more_ws/src
```

## 3. Clone the repository

Clone `more_interfaces` into the `src` folder of your workspace:

```bash
cd ~/more_ws/src

# SSH:
git clone git@github.com:MOREnvironment/more_interfaces.git

# or HTTPS:
git clone https://github.com/MOREnvironment/more_interfaces.git
```

After this, your workspace should look like:

```text
more_ws/
├── src/
│   └── more_interfaces/
├── build/   (created after first build)
└── install/ (created after first build)
```

## 4. Build the package

From the **workspace root**:

```bash
cd ~/more_ws

# Source ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# Build only this package (optional but recommended)
colcon build --packages-select more_interfaces
```

If the build succeeds, you will see something like:

```text
Summary: 1 package finished
```

## 4. Use the package 

To use `more_interfaces`, you must **source both**:

1. ROS 2 Jazzy
2. Your workspace `install` prefix

Open a **new terminal** and run:

```bash
# 1) ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# 2) Your workspace overlay
cd ~/more_ws
source install/setup.bash
```

You can now verify that the package is visible:

```bash
ros2 pkg list | grep more_interfaces
```

Expected output:

```text
more_interfaces
```

### 4.1. Check the interfaces

To see the messages defined in `more_interfaces`:

```bash
ros2 interface list | grep more_interfaces
```

To inspect a specific message (example):

```bash
ros2 interface show more_interfaces/msg/AllocatorResponse
```

This should print the field definitions from the corresponding `.msg` file.

### 4.2. Using the messages in your own code

In a Python node:

```python
from more_interfaces.msg import AllocatorResponse

msg = AllocatorResponse()
# fill in fields...
```

Make sure the environment is sourced (as in step 5) before running your node.

In your CMakeLists.txt of another package:

```cmake
find_package(more_interfaces REQUIRED)

ament_target_dependencies(your_node_target
  rclcpp
  more_interfaces
)
```

Then include the header:

```cpp
#include "more_interfaces/msg/allocator_response.hpp"
```


## 5. Planned maritime-specific interfaces

Over time, this package is intended to collect interfaces that are not covered
by existing ROS 2 packages, in particular:

- Underwater navigation & acoustic sensing
  - DVL (Doppler Velocity Log) data
  - Acoustic positioning fixes (e.g. LBL / USBL / SSBL)
  - Acoustic modem communication (packets, metadata, ranges)

- Maritime zones & risk
  - Maritime operational / legal zones
  - Spatio-temporal risk maps and risk layers relevant to sea operations

- Pipeline / cable following & subsea infrastructure
  - Descriptions of pipelines/cables and their segments
  - Support for inspection or following missions along subsea structures

- Docking, launch & recovery
  - Docking commands and status
  - Launch/recovery sequences for AUVs and ASVs

- Vehicle & fleet state (maritime context)
  - High-level vessel / AUV state focused on maritime modes and constraints
  - Fleet-level task / mission status for cooperating vehicles

## 6. Existing ROS 2 interfaces 

> **This package is designed to extend functionality of existing ROS2 interfaces, not to rewrite the existing standard interfaces. If message, service or action exists you should use it!**

Repositories in which you can find standard ROS2 interfaces:

- `geometry_msgs` (poses, twists, transforms)
- `nav_msgs` (paths, occupancy grids, odometry, planning services)
- `nav2_msgs` (routes, navigation actions such as NavigateToPose, FollowWaypoints, etc.)
- `geographic_msgs` (GeoPoint, GeoPose, GeoPath, RouteNetwork, etc.)
- `unique_identifier_msgs` (UUIDs for missions, tasks, waypoints)
- `action_msgs`, `lifecycle_msgs`, `diagnostic_msgs`
- `std_msgs` and `builtin_interfaces` for primitive types and time/duration
- etc.

The long-term aim is for `more_interfaces` to be a clean, minimal, maritime-focused extension layer on top of these existing ROS 2 interfaces.

## 📝 Credits

Academic Title | Author|GitHub | e-mail | Organization
| :--- | :---: | :---: | :---: | :---:
PhD Candidate| Enio Krizman  | [@kr1zzo](https://github.com/kr1zzo) | enio.krizman@ntnu.no / krizman.enio@outlook.com | NTNU
MsC Student | Audun Bagøien Hustad | [@lordAudun](https://github.com/lordAudun) | audunbh@stud.ntnu.no | NTNU

#### [&copy; Centre of Excellence in Maritime Robotics and Technologies for Sustainable Blue Economy, 2026.](https://marble.eu/)
<img src="assets/marble.png" alt="drawing" width="250"/>`

#### [&copy; Norwegian University of Science and Technology, Department of Marine Technology, 2026.](https://www.ntnu.no/imt)
<img src="assets/ntnu.png" alt="drawing" width="240"/>
