#!/bin/bash

# Define paths
WORLD_SDF="src/uav_description/sdf/world.sdf"
URDF_FILE="src/uav_description/urdf/pioneer.urdf"
GUI_CONFIG="gazebo/gui.config"

# Launch Gazebo server with the specified world and GUI config
echo "Launching Gazebo with world: $WORLD_SDF and GUI config: $GUI_CONFIG..."
ign gazebo $WORLD_SDF --gui-config $GUI_CONFIG &

# Place the Pioneer model within the world
echo "Placing Pioneer model in the world..."
ign service -s /world/pioneer_world/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req "sdf_filename: \"$URDF_FILE\", name: \"pioneer\""

echo "Done!"
