#!/bin/bash

# Determine OS type
OS=$(uname)
echo "Detected OS: $OS"
if [[ "$OS" == "Darwin" ]]; then
    IS_MACOS=true
else
    IS_MACOS=false
fi

# Define variables
IMAGE_NAME="uav-llm-integration"
CONTAINER_NAME="uav-llm-integration-container"
CONF_FILE=".env.conf"
ENV_FILE=".env"

# Check if the .env file exists
if [ ! -f "$ENV_FILE" ]; then
    echo "Error: .env file not found! Aborting."
    exit 1
fi

# For Linux systems, check if a joystick is detected
if [ "$IS_MACOS" = false ]; then
    if [ ! -e "/dev/input/js0" ]; then
        echo "Error: No joystick detected! Please connect a controller before launching."
        exit 1
    fi
fi

# Remove any existing container with the same name
if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm -f $CONTAINER_NAME
fi

# Remove any existing image with the same name
if [ "$(docker images -q $IMAGE_NAME)" ]; then
    echo "Removing existing image: $IMAGE_NAME"
    docker rmi -f $IMAGE_NAME
fi

# Load variables from $CONF_FILE
export $(grep -v '^#' $CONF_FILE | xargs)

# Load the API key from $ENV_FILE
export $(grep -v '^#' $ENV_FILE | xargs)

# Build the Docker image while passing in all necessary build arguments
echo "Building Docker image: $IMAGE_NAME"
docker build \
  --build-arg SAFETY_STOP_DISTANCE=${SAFETY_STOP_DISTANCE} \
  --build-arg MAX_FORWARD_SPEED=${MAX_FORWARD_SPEED} \
  --build-arg MAX_REVERSE_SPEED=${MAX_REVERSE_SPEED} \
  --build-arg MAX_TURN_LEFT_SPEED=${MAX_TURN_LEFT_SPEED} \
  --build-arg MAX_TURN_RIGHT_SPEED=${MAX_TURN_RIGHT_SPEED} \
  --build-arg AREA_THRESHOLD=${AREA_THRESHOLD} \
  --build-arg LLM_URL=${LLM_URL} \
  --build-arg LLM_MODEL=${LLM_MODEL} \
  --build-arg LLM_TEMPERATURE=${LLM_TEMPERATURE} \
  --build-arg LLM_API_INTERVAL=${LLM_API_INTERVAL} \
  --build-arg LLM_PAUSE=${LLM_PAUSE} \
  -t $IMAGE_NAME .

if [ "$IS_MACOS" = false ]; then
    # Allow Docker access to the X server for GUI applications on Linux
    xhost +local:docker
fi

echo "Running the Docker container..."
if [ "$IS_MACOS" = true ]; then
    # macOS: Run container without X11 or device mounts (adjust as needed)
    docker run -it --rm \
        --name $CONTAINER_NAME \
        --env-file $ENV_FILE \
        --privileged \
        $IMAGE_NAME
else
    # Linux: Include X11 settings and device access for joystick and graphics
    docker run -it --rm \
        --name $CONTAINER_NAME \
        --env-file $ENV_FILE \
        -e DISPLAY=$DISPLAY \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e NO_AT_BRIDGE=1 \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --device /dev/dri:/dev/dri \
        --device /dev/ttyUSB0:/dev/ttyUSB0 \
        --device /dev/input:/dev/input \
        --privileged \
        $IMAGE_NAME

    # Reset X server access
    xhost -local:docker
fi