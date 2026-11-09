# Newest Ubuntu + ROS 2 pair that PX4 supports (24.04 + Jazzy, Gazebo Harmonic).
FROM ros:jazzy-ros-base

# mesa-utils: glxinfo to check GPU rendering; also pulls in the OpenGL libs the NVIDIA driver plugs into.
# openssh-client: git over SSH, using the host ssh-agent that VS Code forwards.
RUN apt-get update && apt-get install -y --no-install-recommends mesa-utils openssh-client \
 && rm -rf /var/lib/apt/lists/*

COPY install-px4-deps.sh /tmp/
RUN bash /tmp/install-px4-deps.sh && rm /tmp/install-px4-deps.sh

# cf_drone stack (external/cf_drone, cf_ws) and the experiment tools (tools/):
# - ROS packages for MAVROS, octomap, stereo images, the Gazebo bridge and RViz; egm96-5 geoids for MAVROS
# - OpenGL/EGL libraries for DROAN; NVIDIA VPI 4 for stereo_disparity (from NVIDIA's Jetson repo, x86_64 build)
# - ffmpeg, Xvfb, x11-utils, imagemagick for recorded runs; python3.12-venv for Fast-FoundationStereo
RUN curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor -o /usr/share/keyrings/nvidia-jetson.gpg \
 && echo 'deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.4 main' \
      > /etc/apt/sources.list.d/nvidia-vpi.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-cv-bridge ros-jazzy-depth-image-proc ros-jazzy-pcl-conversions ros-jazzy-mavros ros-jazzy-mavros-msgs \
      ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-rviz2 geographiclib-tools \
      libglfw3-dev libglm-dev libassimp-dev libegl-dev libgl-dev libnvvpi4 vpi4-dev \
      ffmpeg xvfb x11-utils imagemagick python3.12-venv \
 && geographiclib-get-geoids egm96-5 \
 && rm -rf /var/lib/apt/lists/*

ARG HOST_HOME=/home/ubuntu

# Same home path as the host, so the mounted Claude Code config (absolute plugin paths) resolves.
RUN if [ "$HOST_HOME" != /home/ubuntu ]; then usermod -d "$HOST_HOME" -m ubuntu; fi \
 && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
 && echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc
