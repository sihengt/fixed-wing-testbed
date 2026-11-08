#!/usr/bin/env bash
# Install PX4 SITL dependencies (with Gazebo Harmonic) and the Micro XRCE-DDS Agent. Run as root.
set -euo pipefail

# Keep in sync with the external/PX4-Autopilot submodule tag.
PX4_VERSION=v1.17.0
# Agent version from the PX4 ROS 2 user guide.
XRCE_AGENT_VERSION=v2.4.3

tmp=$(mktemp -d)

# PX4's own setup script. It needs requirements.txt next to it.
# --no-nuttx: SITL only, no flight-controller toolchains.
for f in ubuntu.sh requirements.txt; do
  curl -fsSL "https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_VERSION}/Tools/setup/$f" -o "$tmp/$f"
done
# Keep the apt numpy: ROS message bindings are built against it, and pip cannot replace a Debian package.
python3 -c 'import numpy; print("numpy==" + numpy.__version__)' > "$tmp/constraints.txt"
PIP_CONSTRAINT="$tmp/constraints.txt" RUNS_IN_DOCKER=true bash "$tmp/ubuntu.sh" --no-nuttx
# OMPL for fw_planner RRT*: the ROS apt package has no Python bindings, the pip wheel does.
PIP_CONSTRAINT="$tmp/constraints.txt" pip install --break-system-packages ompl==2.0.1

# Bridge between PX4 (uXRCE-DDS client) and ROS 2.
git clone --depth 1 -b "$XRCE_AGENT_VERSION" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$tmp/agent"
mkdir "$tmp/agent/build"
cd "$tmp/agent/build"
cmake ..
make -j"$(nproc)"
make install
ldconfig /usr/local/lib/

cd /
rm -rf "$tmp" /var/lib/apt/lists/*
