#!/bin/bash

# Get workspace root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load common variables (auto-copy from example if not exists)
if [ ! -f "$SCRIPT_DIR/config.sh" ]; then
    cp "$SCRIPT_DIR/config.sh.example" "$SCRIPT_DIR/config.sh"
fi
source "$SCRIPT_DIR/config.sh"

# Check if the image exists
if ! docker image inspect $IMAGE_NAME > /dev/null 2>&1; then
    echo "Error: Image $IMAGE_NAME not found."
    echo "Pull the image first: docker pull $IMAGE_NAME"
    echo "Or build locally: ./build.sh"
    exit 1
fi

# Enable X11 access for Docker
echo "Enabling X11 access for Docker..."
xhost +local:docker

# Run the Docker container
echo "Running Docker container from image: $IMAGE_NAME..."
docker run -it --rm \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    -v $PROJECT_ROOT:/workspace/Gripper_manager \
    -w /workspace/Gripper_manager \
    --name $CONTAINER_NAME \
    $IMAGE_NAME

# Disable X11 access after container exit
echo "Disabling X11 access after container exit..."
xhost -local:docker
