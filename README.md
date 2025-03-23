# uav-llm-integration

**Conan Dewitt (22877792) [GitHub](https://github.com/Ayodhya27)**

This project integrates unmanned autonomous vehicles (UAVs) with large language model (LLM) capabilities, enhancing operator-machine interaction through intelligent decision-making and real-time data analysis. The repository contains a ROS2 project that can be run in either pure simulation mode, or live demonstration mode - onboard a Pioneer 3AT, providing a robust framework for UAV control and experimentation.

## Installation

## Getting Started

### Running the Project

Ensure that Docker is installed and a DualShock 4 controller is connected (wired or wireless) before using the run script. An OpenAI API key is also required for launch, and should be placed within an `.env` file under the `LLM_API_KEY=` variable.
Note that this project has only been tested on Linux systems (As of 13/03/2025) and requires a moderately powerful system to run the simulation.


Use the run script to build, and launch the container - this will take a while the first time:

```bash
./run_uav_llm_integration.sh
```

If you don't have access to a DualShock 4 controller, you can bypass this requirement by commenting out lines 23 - 30 (As of 13/03/2025).


Unless you have access to a Pioneer 3AT UAV, it can be assumed you will be running this project exclusively in simulation mode. The simulation can be started with:

```bash
ros2 launch master_launch sim.launch.py
```

Pressing up right after launch, the start of the launch command will populate your terminal for you.

### Running on a Pioneer

If you do have access to a Pioneer, you can instead use:

```bash
ros2 launch master_launch actual.launch.py
```

Which will launch this project on the actual Pioneer hardware instead of within a simulation.

### Known Issues

- If you have any issues with communication between the Docker Container and connected hardware, check the Useful Commands section for permission passthroughs.
- During the first launch of the simulation, after a fresh build, you might find that the dummy instance of Gazebo won't render in properly and doesnt close itself, thus, you can safely close the dummy instance yourself. The dummy instance was implemented as workaround to fix the issue of the actual Gazebo instance not launching properly upon loading a fresh build.

### Useful Commands

Check what index the usb serial controller is (Pionner 3AT drivetrain control).

```bash
ls /dev/ttyUSB*
```

Allocate the LiDAR an address

```bash
sudo ip addr add 192.168.0.100/24 dev enp89s0
```

Allow the webcam to communicate with the container.

```bash
sudo chmod 666 /dev/video0
```

Check which event the DualShock 4 gamepad publishes to.

```bash
sudo evtest
```

Enable the event to be read by the container (A DualShock 4 controller is exclusively scanner for by the code).

```bash
sudo chmod 666 /dev/input/event*
```