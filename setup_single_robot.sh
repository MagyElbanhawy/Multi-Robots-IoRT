#!/bin/bash
# EMRMF Single Robot Setup Script (Phase 1: Decentralized Local Mapping)
# OS: Ubuntu 22.04 LTS
# ROS Version: ROS 2 Humble

set -e

echo "======================================================="
echo "1. System Update & Locale Setup"
echo "======================================================="
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales software-properties-common curl gnupg2 lsb-release git build-essential cmake
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "======================================================="
echo "2. Install ROS 2 Humble"
echo "======================================================="
# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "======================================================="
echo "3. Create ROS 2 Workspace"
echo "======================================================="
mkdir -p ~/emrmf_ws/src
cd ~/emrmf_ws/src

echo "======================================================="
echo "4. Install Hardware Drivers (EAI T-mini Pro & Orbbec DaBai)"
echo "======================================================="

# --- Install YDLidar SDK (Prerequisite for EAI T-mini Pro) ---
cd ~/emrmf_ws
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install
cd ~/emrmf_ws/src

# --- Install YDLidar ROS 2 Driver ---
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git

# --- Install Orbbec SDK ROS 2 Wrapper (For DaBai RGB-D Camera) ---
git clone -b humble https://github.com/orbbec/OrbbecSDK_ROS2.git

echo "======================================================="
echo "5. Install SLAM, Localization & Communication Dependencies"
echo "======================================================="
sudo apt install -y \
    ros-humble-rtabmap-ros \
    libg2o-dev \
    ros-humble-imu-tools \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    ros-humble-serial-driver \
    python3-serial

echo "======================================================="
echo "6. Build the Workspace"
echo "======================================================="
cd ~/emrmf_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source ~/emrmf_ws/install/setup.bash
if ! grep -q "source ~/emrmf_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/emrmf_ws/install/setup.bash" >> ~/.bashrc
fi

echo "======================================================="
echo "Setup Complete!"
echo "Please close this terminal or run 'source ~/.bashrc' before proceeding."
echo "======================================================="
