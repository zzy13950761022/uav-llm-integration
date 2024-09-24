# uav-llm-integration
MPE Research Project @ The University of Western Australia by Conan (Po) Dewitt

## Installing ROS 2 and Gazebo for Simulation
This project utilises [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/). ROS is the foundational framework that provides the functionality needed for the UAV application and supports integration for simulated testing. Gazebo is a simulation environment that works with ROS, allowing the ROS packages to be tested within a realistic physics-based simulation.

### Installing ROS 2 Humble
Enable the required ROS repositories:
```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2:
```sh
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

Install the required packages for this project:
```sh
./install_packages.sh
```

Finally source the ROS installation:
```sh
source /opt/ros/humble/setup.bash
```

### Installing Gazebo Fortress
Enable the required Ignition repositories:
```sh
sudo apt install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

Install Ignition Fortress:
```sh
sudo apt update
sudo apt upgrade
sudo apt install ignition-fortress
```

## Running Simulations
These following instructions will be the bulk of the implementation, that is unless, access to a Pioneer 3AT UAV exists.

### Quick test in Gazebo
Launch Gazebo by running included the shell script:
```sh
./launch_gazebo.sh
```
This should open up Gazebo, with the Pioneer placed. Refresh both the Camera and LIDAR topics in the right-hand-side menu after launching. At the bottom of said menu is also a manual override to control the UAV.

To launch Gazebo manually, do the following;

Firstly, launch a Gazebo server with the specified simulation world using:
```sh
ign gazebo src/uav_description/sdf/world.sdf
```

Then in a second terminal, place the Pioneer within that world with:
```sh
ign service -s /world/pioneer_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "src/uav_description/urdf/pioneer.urdf", name: "pioneer"'
```

To access the camera, LIDAR, and manual controls in Gazebo, go to the top-right-hand chevron and add `Image Display`, `Visualize Lidar`, and `Teleop` respectively.

LIDAR data can be directly printed to the terminal using:
```sh
ign topic -e --topic /lidar
```

A list of all available topics can be printed with:
```sh
ign topic -l
```

### Running Gazebo and RVIZ
Firstly, build the entire project with:
```sh
colcon build
```

Next, source the workspace:
```sh
source install/setup.bash
```

And finally, launch the RVIZ simulation package `uav_description`:
```sh
ros2 launch uav_description simulation.launch.py
```