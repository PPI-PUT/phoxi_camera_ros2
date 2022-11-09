rocker --network host --privileged --nvidia --x11 --user --name phoxi \
    --env="USER" \
    --env=RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --volume "${HOME}/phoxi_ws:${HOME}/phoxi_ws" \
    --volume /dev/shm \
    --volume ${HOME}/.vscode:${HOME}/.vscode-server \
    -- amadeuszsz/phoxi:humble
