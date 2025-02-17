#!/bin/bash

# Define the image name
IMAGE_NAME="uav-llm-integration"
CONTAINER_NAME="uav-llm-integration-container"
ENV_FILE=".env"

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
docker build --build-arg LLM_API_KEY=$(grep LLM_API_KEY "$ENV_FILE" | cut -d '=' -f2) -t $IMAGE_NAME .

# Allow Docker access to the X server for GUI applications
xhost +local:docker

# Run the container
echo "Running the Docker container..."
docker run -it --rm \
    --name $CONTAINER_NAME \
    --env-file $ENV_FILE \
    -e DISPLAY=$DISPLAY \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=mesa \
    -e NO_AT_BRIDGE=1 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    $IMAGE_NAME

# Reset X server access
xhost -local:docker
