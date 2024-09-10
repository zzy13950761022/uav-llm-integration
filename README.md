# uav-llm-integration
MPE Research Project @ The University of Western Australia by Conan (Po) Dewitt

## Installing ROS 2 and RVIZ for Simulation
This simulation implementation uses Tobias Fischer et al.'s [RoboStack](https://robostack.github.io/index.html), a bundling of ROS packages using the Conda package manager.

### Installing Mamba
It should be noted here that the default Anaconda installer should *not* be used here; [Miniforge](https://github.com/conda-forge/miniforge) is the preferred installer.

Once Miniforge has been installed, Conda can be installed with:
```sh
conda install mamba -c conda-forge
```

### Installing ROS 2
Create a virtual environment:
```sh
mamba create -n uav-llm-integration-env python=3.11
```

Initialise the virtual environment:
```sh
mamba init <zsh>
```
If on macOS and using zshell, include `zsh` at the end of the line, otherwise bash is assumed as the default shell.

Activate virtual environment:
```sh
mamba activate uav-llm-integration-env
```

Configure channels:
```sh
# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, THIS MIGHT RETURN AN ERROR if it is not in the list WHICH IS OK
conda config --env --remove channels defaults
```

Install ROS 2 Humble:
```sh
mamba install ros-humble-desktop
```

Restart the environment:
```sh
mamba deactivate
mamba activate uav-llm-integration-env
```

### Installing development tools
Required tools:
```sh
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

Other tools can be installed using:
```sh
mamba install <tool_of_choice>
```

### Installing Ignition Gazebo
As of now, this integration of Gazebo only works on Linux and NOT MacOS (Windows is untested). This step is optional, however, heavily desired as the Gazebo simulation depicts what is happening to the UAV within an external physics engine, as opposed to the RVIZ simulation, which depicts what the UAV believes is happening, which is unreliable for real-world simulation.

Install Ignition Fortress as instructed by [Gazebo](https://gazebosim.org/docs/fortress/install_ubuntu/).

## Running Simulations
These following instructions will be the bulk of the implementation, that is unless, access to a Pioneer 3AT UAV exists.

### Running in Gazebo
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

### Running in RVIZ