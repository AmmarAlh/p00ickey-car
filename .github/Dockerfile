FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=jazzy \
    ROS_VERSION=2

# --------------------------------------
# 1. Base OS setup
# --------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    ca-certificates \
    gnupg2 \
    lsb-release \
    curl \
    wget \
    git \
    build-essential \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------
# 2. Add ROS repository & keys
# --------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# --------------------------------------
# 3. Install ROS base packages and tools
# --------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-launch-testing \
    ros-${ROS_DISTRO}-ament-lint \
    ros-dev-tools \
    python3 \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------
# 4. Create isolated Python environment
# --------------------------------------
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# --------------------------------------
# 5. Install Python tools in venv
# --------------------------------------
RUN pip install --no-cache-dir \
    pre-commit \
    coverage \
    pytest \
    flake8 \
    black \
    isort \
    cpplint \
    colcon-common-extensions

# --------------------------------------
# 6. ROS environment config (for login shells)
# --------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# --------------------------------------
# 7. Entrypoint + Working directory
# --------------------------------------
COPY .devcontainer/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /root/p00ickey-car
