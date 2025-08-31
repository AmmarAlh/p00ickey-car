# 🧠 Pickey-Car

This is a weekend project I’m doing for fun and to stay up-to-date with ROS 2. The Pickey-Car is basically a car with a 9-degree-of-freedom robot hand strapped to the top. I’m building everything for both **simulation** and **real hardware** environments.

---

## 📁 Workspace Structure
```
ros\_ws/
└── src/
├── pickeycar\_bringup/         # Central launch and config to start the full system (sim or real)
│   ├── launch/
│   │   ├── sim/               # Launch files for simulation setup
│   │   └── real/              # Launch files for the real robot
│   └── config/                # Shared parameters for sim/real
├── pickeycar\_description/     # URDF/Xacro, meshes, and other robot-specific config
│   ├── urdf/                  # Robot model definitions
│   ├── meshes/                # 3D models for RViz and simulation
│   └── config/                # Description-related parameters
├── pickeycar\_gazebo/          # Gazebo simulation setup
│   ├── worlds/                # Custom simulation worlds
│   ├── models/                # Additional Gazebo models
│   ├── plugins/               # Custom or external plugins
│   └── launch/                # Simulation launch logic
├── pickeycar\_navigation/      # Nav2 stack config and maps
│   ├── config/                # Navigation + behavior tree parameters
│   └── maps/                  # Maps for SLAM and localization
├── pickeycar\_control/         # Low-level motion control and PID logic
│   ├── src/                   # Control node source code
│   └── config/                # Parameters (PID, limits, etc.)
├── pickeycar\_hand/            # (Optional) Robot hand description and control
│   ├── description/           # Hand URDF/Xacro and meshes
│   ├── control/               # Hand-specific controllers
│   └── config/                # Hand parameters
├── pickeycar\_interfaces/      # Custom message/service/action definitions
└── pickeycar\_apps/            # High-level behaviors: cleaning, mapping, patrol, etc.
├── scripts/               # App logic/scripts
└── config/                # App-specific parameters
```

---

## 🧩 Simulation vs. Real Hardware

Everything is designed to support both simulation and real-world testing. Each major system can be toggled via launch arguments and shared configs.

| Component          | Simulation                          | Real Robot                           |
|--------------------|-------------------------------------|--------------------------------------|
| URDF & TF          | Shared                              | Shared                               |
| Sensor Input       | Gazebo plugins (e.g., lidar, IMU)   | Real drivers (serial/USB)            |
| Motion Control     | Gazebo diff-drive plugin            | Roomba serial interface              |
| Navigation (Nav2)  | Shared (with sim maps)              | Shared (with real SLAM/maps)         |
| Launch Files       | `sim.launch.py`                     | `real.launch.py`                     |

---

## 🧠 Package Overview

- **pickeycar_description**: URDF/Xacro, meshes, TF config. Shared across sim and real.
- **pickeycar_gazebo**: Worlds, robot spawner, simulated sensors.
- **pickeycar_control**: Motion commands → wheels (diff-drive or serial).
- **pickeycar_navigation**: Nav2 stack setup, behavior trees, and maps.
- **pickeycar_bringup**: Entry-point launches — set up full sim or real stack.
- **pickeycar_apps**: Logic for room cleaning, patrol, mapping, etc.
- **pickeycar_hand** (optional): Control stack for the robot hand.
- **pickeycar_interfaces**: ROS messages, services, and actions if needed.

---

## 🚀 Getting Started

### Build the workspace

```bash
cd ros_ws
colcon build
source install/setup.bash
```

### Run in simulation

```bash
ros2 launch pickeycar_bringup sim.launch.py
```

### Run on real robot

```bash
ros2 launch pickeycar_bringup real.launch.py
```

---

## 🧪 Dev Tips

* Use `use_sim_time:=true` for simulation time sync.
* Namespaces (`/pickeycar`, `/robot1`, etc.) are useful for multi-robot setups.
* Try to keep URDFs and parameters consistent across sim and real.
* Use RViz config files to keep your debugging setup quick.
* Tools like `ros2 topic`, `ros2 param`, and `rqt` help a lot when debugging.

---

## 🧰 Optional Tools

* `ros2_control` (future): cleaner hardware interface abstraction.
* `rqt` for GUI-based debugging.
* `rosbag2` for recording and replaying real/sim data.

---

## 📌 Next Steps


* [ ] Find a good LiDAR
* [ ] Select a RealSense Camera might be D345
* [ ] Ackermann steering chassis 
* [ ] Finalize the URDF for the Roomba + hand
* [ ] Integrate simulated lidar and IMU in Gazebo
* [ ] Hook up serial comms to the real Roomba
* [ ] Translate `cmd_vel` into Roomba-compatible commands
* [ ] Add Nav2 setup + mapping capabilities

