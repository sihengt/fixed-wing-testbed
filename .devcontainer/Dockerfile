# Newest Ubuntu + ROS 2 pair that PX4 supports (24.04 + Jazzy, Gazebo Harmonic).
FROM ros:jazzy-ros-base

# mesa-utils: glxinfo to check GPU rendering; also pulls in the OpenGL libs the NVIDIA driver plugs into.
# openssh-client: git over SSH, using the host ssh-agent that VS Code forwards.
RUN apt-get update && apt-get install -y --no-install-recommends mesa-utils openssh-client \
 && rm -rf /var/lib/apt/lists/*

COPY install-px4-deps.sh /tmp/
RUN bash /tmp/install-px4-deps.sh && rm /tmp/install-px4-deps.sh

ARG HOST_HOME=/home/ubuntu

# Same home path as the host, so the mounted Claude Code config (absolute plugin paths) resolves.
RUN if [ "$HOST_HOME" != /home/ubuntu ]; then usermod -d "$HOST_HOME" -m ubuntu; fi \
 && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
 && echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc
