#!/bin/bash

# Build the Docker image
docker build -t f110-humble:latest .

# Run the Docker container
docker run -it \
    --name f110-dev \
    --gpus all \
    --network=host \
    --device=/dev/input \
    -e NVIDIA_DRIVER_CAPABILITIES=all,graphics \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/kimkh/f110_ws:/f110_ws \
    f110-humble:latest
# Note: --rm flag will automatically clean up the container after exit