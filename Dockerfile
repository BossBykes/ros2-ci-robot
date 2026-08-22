# ROS 2 Jazzy development image for the ros2-ci-robot project.
# All ROS 2 / Gazebo dependencies live inside this container.
FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---------------------------------------------------------------------------
# Development, ROS 2, Gazebo and testing dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    gdb \
    wget \
    curl \
    nano \
    vim \
    less \
    sudo \
    lcov \
    gcovr \
    clang-format \
    clang-tidy \
    python3-pip \
    python3-venv \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-colcon-clean \
    python3-rosdep \
    python3-vcstool \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-timeout \
    python3-pytest-mock \
    ros-jazzy-ros-gz \
    ros-jazzy-launch-testing \
    ros-jazzy-launch-testing-ros \
    ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-launch-pytest \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-gmock \
    ros-jazzy-ament-cmake-pytest \
    ros-jazzy-ament-cmake-test \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-ament-cmake-copyright \
    ros-jazzy-ament-cmake-cppcheck \
    ros-jazzy-ament-cmake-lint-cmake \
    ros-jazzy-ament-cmake-uncrustify \
    ros-jazzy-ament-cmake-xmllint \
    ros-jazzy-ament-index-python \
    ros-jazzy-ros2cli-common-extensions \
    ros-jazzy-rosbag2 \
    ros-jazzy-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# The ROS Noble base image already contains the "ubuntu" user with UID/GID
# 1000. Reuse it instead of deleting/recreating users.
# ---------------------------------------------------------------------------
RUN mkdir -p /workspace \
    && chown "${USER_UID}:${USER_GID}" /workspace \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu

# ---------------------------------------------------------------------------
# rosdep setup
# ---------------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true

USER ubuntu

RUN rosdep update || true

# Automatically source ROS 2 and this workspace when opening a shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc \
    && echo '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' >> /home/ubuntu/.bashrc

# Project-specific ROS domain keeps this project isolated from other ROS work.
ENV ROS_DOMAIN_ID=77 \
    RCUTILS_COLORIZED_OUTPUT=1 \
    QT_QPA_PLATFORM=offscreen \
    GZ_SIM_HEADLESS=1 \
    COLCON_HOME=/workspace/.colcon

USER root

COPY --chown=1000:1000 docker/entrypoint.sh /usr/local/bin/entrypoint.sh

RUN chmod +x /usr/local/bin/entrypoint.sh

USER ubuntu

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["bash"]
