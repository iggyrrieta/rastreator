#!/bin/bash

# Copyright 2020
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Iñaki Lorente


# Install ROS2 Eloquent & GAZEBO (basics for Simulation)
function usage
{
    echo "Usage: ./rastreator_install.sh [[-p package] | [-h]]"
    echo "Install dependencies to work with repository RASTREATOR"
}

echo "====================================="
echo "RASTREATOR ROS2 DIFF WHEEL ROBOT"
echo "====================================="
# INSTALL ROS2
while true; do
    read -p "Do you wish to install ELOQUENT distribution for ROS2?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "=====================================";
		echo "Adding repository and source list";
		echo "=====================================";
		# SETUP LOCAL
		sudo locale-gen en_US en_US.UTF-8;
		sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8;
		export LANG=en_US.UTF-8;

		# SETUP SOURCES
		sudo apt update && sudo apt install curl gnupg2 lsb-release;
		curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -;
		sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list';

		# Update
		echo "==================";
		echo "Updating apt-get";
		echo "==================";
		sudo apt update;
		echo "====================";
		echo "Installing ROS2 Eloquent";
		echo "====================";
		sudo apt install ros-eloquent-desktop; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
# INSTALL GAZEBO
while true; do
    read -p "Do you wish to install GAZEBO for simulation purpose?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "====================";
		echo "Installing GAZEBO9";
		echo "====================";
		sudo apt-get install gazebo9; 
		sudo apt install ros-eloquent-gazebo-ros-pkgs; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
while true; do
    read -p "Do you wish to install DYNAMIXEL SDK (only need it for motors)?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "==========================";
		echo "Installing DYNAMIXEL SDK";
		echo "==========================";
		sudo apt install ros-eloquent-dynamixel-sdk; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
while true; do
    read -p "Do you wish to install Navigation2 package?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "==========================";
		echo "Installing NAVIGATION2";
		echo "==========================";
		sudo apt install ros-eloquent-navigation2;
		sudo apt install ros-eloquent-nav2-bringup;
		sudo apt install ros-eloquent-cartographer-ros;
		sudo apt install ros-eloquent-slam-toolbox; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
echo "=================================="
echo "Installing packages-dependencies"
echo "=================================="
# INSTALL PACKAGES AND DEPENDENCIES
sudo apt-get install python3-colcon-common-extensions
sudo apt-get install v4l-utils
sudo apt-get install python-rosdep2
echo "END"
echo "=====================================================";
echo "=================================="
echo "Installing Python-dependencies"
echo "=================================="
# INSTALL PACKAGES AND DEPENDENCIES
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
# install some pip packages needed for rastreator  
pip3 install catkin_pkg
pip3 install empy
pip3 install lark
pip3 install numpy
sudo apt install python3-sympy
pip3 install PySide2
pip3 install MatPlotLib
pip3 install xacro
echo "END"
echo "=====================================================";
# THE END
echo "======================================================"
echo "Installation completed!"
echo "- Next step, get the rastreator repository"
echo "git clone https://github.com/iggyrrieta/rastreator"
echo "======================================================"
