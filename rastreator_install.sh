#!/bin/bash
# Install ROS2 Eloquent & GAZEBO (basics for Simulation)

function usage
{
    echo "Usage: ./rastreator_install.sh [[-p package] | [-h]]"
    echo "Install dependencies to work with repository RASTREATOR"
}

echo "====================================="
echo "Adding repository and source list"
echo "====================================="
# SETUP LOCAL
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# SETUP SOURCES
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update
echo "=================="
echo "Updating apt-get"
echo "=================="
sudo apt update

# INSTALL ROS2
echo "=========================="
echo "Installing ROS2 Eloquent"
echo "=========================="
sudo apt install ros-eloquent-desktop

# INSTALL GAZEBO
echo "===================="
echo "Installing GAZEBO9"
echo "===================="
sudo apt install ros-eloquent-desktop

echo "=================================="
echo "Installing packages-dependencies"
echo "=================================="
# INSTALL PACKAGES AND DEPENDENCIES
sudo apt-get install python3-colcon-common-extensions
sudo apt-get install ros-eloquent-gazebo-ros-pkgs

echo "=================================="
echo "Installing Python-dependencies"
echo "=================================="
# INSTALL PACKAGES AND DEPENDENCIES
pip install catkin_pkg
pip install empy
pip install lark
pip install numpy
sudo apt install python3-sympy
pip install PySide2
pip install MatPlotLib

# THE END
echo "======================================================"
echo "Installation completed!"
echo "- Next step, get the rastreator repository"
echo "git clone -b simulation https://github.com/iggyrrieta/rastreator"
echo "======================================================"
