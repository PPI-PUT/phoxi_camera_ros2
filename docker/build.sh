DOCKER_BUILDKIT=1 docker build --network=host --no-cache \
    --build-arg ROS_DISTRO=humble \
    -t amadeuszsz/phoxi:humble .
