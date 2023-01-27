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


HEIGHT=15
WIDTH=60
CHOICE_HEIGHT=10
TITLE="DEPENDENCIES"
BACKTITLE="RASTREATOR"
MENU="What to install:"

OPTIONS=(1 "ROS2 Foxy"
         2 "Gazebo 11"
         3 "Rplidar ROS2 package"
         4 "Dynamixel motors libraries"
         5 "ROS2 Navigation stack and SlamToolbox"
         6 "Camera dependencies"
         7 "Set USB Rules")

CHOICE=$(dialog --clear \
                --backtitle "$BACKTITLE" \
                --title "$TITLE" \
                --menu "$MENU" \
                $HEIGHT $WIDTH $CHOICE_HEIGHT \
                "${OPTIONS[@]}" \
                2>&1 >/dev/tty)

clear

# INSTALL ROS2
install_ros2() 
{
while true; do
    read -p "Do you wish to install FOXY distribution for ROS2?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "=====================================";
		echo "Adding repository and source list";
		echo "=====================================";
		# SETUP LOCAL
		sudo apt update && sudo apt install locales;
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
		echo "Installing ROS2 FOXY";
		echo "====================";
		sudo apt install -y ros-foxy-desktop; 
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
        python3-sympy \
        wget
        # install some pip packages needed for rastreator  
        pip install catkin_pkg 
        pip install empy 
        pip install lark 
        pip install numpy 
        pip install PySide2
        pip install xacro
        echo "END"
        echo "=====================================================";
        break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

# INSTALL GAZEBO
install_gazebo() 
{
while true; do
    read -p "Do you wish to install GAZEBO11 for simulation purpose?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "====================";
		echo "Installing GAZEBO11";
		echo "====================";
		sudo apt install -y gazebo11; 
		sudo apt install -y ros-foxy-gazebo-ros-pkgs; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

install_dynamixel() 
{
while true; do
    read -p "Do you wish to install DYNAMIXEL SDK (only need it for motors)?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "==========================";
		echo "Installing DYNAMIXEL SDK";
		echo "==========================";
        sudo cp usb_rules/wheelmotors.rules  /etc/udev/rules.d;
        sudo service udev reload;
        sudo service udev restart;
		sudo apt install -y ros-foxy-dynamixel-sdk; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

install_navigation() 
{
while true; do
    read -p "Do you wish to install Navigation2 package?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "==========================";
		echo "Installing NAVIGATION2";
		echo "==========================";
		sudo apt install -y ros-foxy-navigation2;
		sudo apt install -y ros-foxy-nav2-bringup; 
        sudo apt install -y ros-foxy-slam-toolbox; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

install_rplidar() 
{
while true; do
    read -p "Do you wish to install rplidar package?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "==========================";
		echo "Installing RPLIDAR";
		echo "==========================";
        sudo cp usb_rules/rplidar.rules  /etc/udev/rules.d;
        sudo service udev reload;
        sudo service udev restart;
        sudo apt install -y ros-foxy-rplidar-ros; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

install_camera() 
{
while true; do
    read -p "Do you wish to install camera dependencies?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "===============================";
		echo "Installing CAMERA dependencies";
		echo "===============================";
        sudo apt install -y v4l-utils;
        sudo apt install -y ros-foxy-image-transport-plugins; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

install_cyclonedds() 
{
while true; do
    read -p "Do you wish to install rmw cyclone dds?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "===============================";
		echo "Installing CYCLONE DDS";
		echo "===============================";
        sudo apt install --no-install-recommends -y libcunit1-dev;
        sudo apt-get install -y ros-foxy-cyclonedds; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

set_usb_rules()
{
    while true; do
    read -p "Do you wish to set USB rules?(y/n): " yn
    case $yn in
        [Yy]* ) 
		echo "===============================";
		echo "SET USB RULES";
		echo "===============================";
        echo -e "remapping the device serial port(ttyUSB*) with permissions to work..."
        sudo cp usb_rules/rplidar.rules  /etc/udev/rules.d;
        sudo cp usb_rules/wheelmotors.rules  /etc/udev/rules.d;
        echo -e "Restarting udev..."
        sudo service udev reload;
        sudo service udev restart;
        echo -e "\nUDEV RULES created!\n"
        break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
echo "END"
echo "=====================================================";
}

case $CHOICE in
        1)
            install_ros2
            ;;
        2)
            install_gazebo
            ;;
        3)
            install_rplidar
            ;;
        4)
            install_dynamixel
            ;;
        5)
            install_navigation
            ;;
        6)
            install_camera
            ;;
        7)
            set_usb_rules
            ;;
esac