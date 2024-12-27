#!/bin/bash

# Define the image name
IMAGE_NAME="uav-llm-integration"
CONTAINER_NAME="uav-llm-integration-container"

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

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME"
docker build -t $IMAGE_NAME .

# Allow Docker access to the X server for GUI applications
xhost +local:docker

# Run the container
echo "Running the Docker container..."
docker run -it --rm \
    --name $CONTAINER_NAME \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri \
    $IMAGE_NAME

# Reset X server access
xhost -local:docker
