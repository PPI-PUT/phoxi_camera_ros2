FROM ubuntu:jammy
ARG ROS_DISTRO

# install packages & setup locales
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    apt-utils \
    build-essential \
    ca-certificates \
    curl \
    dirmngr \
    gnupg2 \
    locales \
    lsb-release \
    keyboard-configuration \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales

ENV LANG="en_US.UTF-8" \
    LANGUAGE="en_US.UTF-8" \
    LC_ALL="en_US.UTF-8"


# Install ROS2 & init rosdep
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    python3-colcon-common-extensions \
    python3-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep init'


# install Photoneo
COPY PhotoneoPhoXiControlInstaller-1.9.1-Ubuntu20-STABLE.tar.gz /tmp/
RUN apt-get update && apt-get install -y --no-install-recommends \
    avahi-daemon \
    libavahi-client3 \
    libgtkglextmm-x11-1.2-0v5 \
    libgomp1 \
    libtiff5 \
    libxcursor1 \
    libxinerama1 \
    libxrandr2 \
    qt5dxcb-plugin \
    zip \
    && tar -xvf "/tmp/PhotoneoPhoXiControlInstaller-1.9.1-Ubuntu20-STABLE.tar.gz" -C /tmp/ \
    && "/tmp/PhotoneoPhoXiControlInstaller-1.9.1-Ubuntu20-STABLE.run" --accept \
    && rm -rf /tmp/*

ENV PHOXI_CONTROL_PATH="/opt/Photoneo/PhoXiControl-1.9.1"


# install phoxi camera ros2 driver dependencies (for later time-saving purposes with rosdep)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    libpcl-dev \
    libqt5svg5-dev \
    ros-humble-ament-cmake-auto \
    ros-humble-cv-bridge \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-pcl-conversions \
    ros-humble-rclcpp \
    ros-humble-rclcpp-components \
    ros-humble-rclcpp-lifecycle \
    ros-humble-robot-state-publisher \
    ros-humble-rosidl-default-runtime \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-xacro


# customize bash
COPY bashrc-git-prompt /tmp/
RUN curl -L https://raw.github.com/git/git/master/contrib/completion/git-prompt.sh >> /etc/skel/.bashrc \
    && cat /tmp/bashrc-git-prompt >> /etc/skel/.bashrc \
    && rm -rf /tmp/bashrc-git-prompt \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc


COPY entrypoint /entrypoint
ENTRYPOINT ["/entrypoint"]
CMD ["/bin/bash"]
LABEL maintainer="Amadeusz Szymko (amadeuszszymko@gmail.com)"
