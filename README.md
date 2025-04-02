# uav-llm-integration

**Conan Dewitt (22877792) [GitHub](https://github.com/conanpodewitt)**

This project integrates unmanned autonomous vehicles (UAVs) with large language model (LLM) capabilities, enhancing operator-machine interaction through intelligent decision-making and real-time data analysis. The repository contains a ROS 2 project that can be run in either pure simulation mode or live demonstration mode on a Pioneer 3AT, providing a robust framework for UAV control and experimentation.

## Installation

## Getting Started

### Running the Project

Ensure that Docker is installed and a DualShock 4 controller is connected (wired or wireless) before using the run script. An OpenAI API key is also required for launch and should be placed within an `.env` file under the `LLM_API_KEY=` variable.

> **Note:** This project has only been tested on Linux systems (as of 13/03/2025) and requires a moderately powerful system to run.

Use the run script to build and launch the container — this will take some time on the first run:

```bash
./run.sh
```

If you don't have a DualShock 4 controller, you can simply disable the check by commenting out lines 31-35 in the script (as of 02/04/2025). Note that while this change prevents teleoperation on a Pioneer 3AT, you can still control the simulated robot via Gazebo.

### Simulation

Unless you have access to a Pioneer 3AT UAV, it is assumed that you will be running this project exclusively in simulation mode. The simulation can be started with:

```bash
ros2 launch master_launch sim.launch.py
```

Pressing the up arrow (`↑`) right after launch will populate the terminal with a partially completed launch command.

### Pioneer 3AT

If you have access to a Pioneer 3AT, you can launch the project on the actual hardware using:

```bash
ros2 launch master_launch actual.launch.py
```

This will start the system on the real Pioneer instead of in simulation.

### Known Issues

If you experience communication issues between the Docker container and connected hardware, check the [Useful Commands](#useful-commands) section for permission passthroughs.
- On the initial simulation launch following a fresh build, Gazebo may not render correctly — simply restart the container to resolve the issue.

## Useful Commands

### Check USB Serial Connection (Pioneer 3AT drivetrain control)
```bash
ls /dev/ttyUSB*
```

### Grant Permission for USB Serial Communication
```bash
sudo chmod 666 /dev/ttyUSB*
```

### Assign an IP Address to the LiDAR
```bash
sudo ip addr add 192.168.0.100/24 dev enp89s0
```

### Allow Webcam Communication with the Container
```bash
sudo chmod 666 /dev/video0
```

### Check Which Event the DualShock 4 Controller Publishes To
```bash
sudo evtest
```

### Enable Event Access for the DualShock 4 Controller
```bash
sudo chmod 666 /dev/input/event*
```