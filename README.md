# uav-llm-integration
MPE Research Project @ The University of Western Australia by Conan (Po) Dewitt

## Installing ROS 2 and RVIZ for Simulation
This simulation implementation uses Tobias Fischer et al.'s [RoboStack](https://robostack.github.io/index.html), a bundling of ROS packages using the Conda package manager.

#### Installing Mamba
It should be noted here that the default Anaconda installer should *not* be used here; [Miniforge](https://github.com/conda-forge/miniforge) is the preferred installer.

Once Miniforge has been installed, Conda can be installed with:
```sh
conda install mamba -c conda-forge
```

#### Installing ROS 2
Create a virtual environment:
```sh
mamba create --prefix uli-env python=3.11
```

Initialise the virtual environment:
```sh
mamba init <zsh>
```
If on macOS and using zshell, include `zsh` at the end of the line, otherwise bash is assumed as the default shell.

Activate virtual environment:
```sh
mamba activate ./uli-env
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
mamba activate uli-env
```

#### Installing development tools
Default tools:
```sh
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

Other tools can be installed using:
```sh
mamba install <tool_of_choice>
```