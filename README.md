# Rastreator
ROS2 differential wheel robot. Master branch is ROS2 Foxy distribution

Please check [requirements](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/ROS2%20Foxy%20requirements.md). This requirements are the ones we are going to install, but take in consideration the `Python` requirement. You must install a `Python` version equal or greater than 3.8

## Installation



### Method 1: Automatic

Open a new terminal and go to the repository root folder:

```bash
sudo chmod +x installer/rastreator_install.sh
```

```bash
.installer/rastreator_install.sh
```

This will install all `Ros2` and `Gazebo11` apt packages and also all `Python` dependencies.

When the installation ends, copy the following in your `~/.bashrc` file:


[Create ROS2/GAZEBO11 local variables](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/Local%20variables%20for%20ROS2%20and%20Gazebo.md)



### Method 2: Step by step

1. [Install ROS2 eloquent](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/ROS2%20Foxy%20installation.md)

2. [Install GAZEBO11](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/Gazebo%20installation.md)

3. [Create ROS2/GAZEBO11 local variables](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/Local%20variables%20for%20ROS2%20and%20Gazebo.md)

4. Install python dependencies using `pip`

   Go to repository root folder and run in a terminal:

   `pip install -r installer/simulation_requirements.txt`
   
5. Instal the following apt dependencies:

   `sudo apt-get install v4l-utils`
   
   `sudo apt-get install ros-foxy-navigation2`
   
   `sudo apt-get install ros-foxy-nav2-bringup`
   
   `sudo apt-get install ros-foxy-image-transport-plugins`
   
   `sudo apt-get install ros-foxy-dynamixel-sdk`

   `sudo apt-get install python-rosdep2`

   `sudo apt install python3-sympy`



## Build

```bash
colcon build --symlink-install
```

Source the compilation if you want to test a package using same terminal:

```bash
source install/local_setup.bash
```



## Remove all ROS related

> Just in case you want to remove all ROS installed (be carefull this will remove all previous installation)

```bash
sudo apt remove ros-* && sudo apt autoremove
```



## Usage



### Manual control

Terminal 1 -> `ros2 launch rastreator_bringup start_manual.launch.py`

Terminal 2 -> `rviz2` (file/open Config/{path/to/rastreator_description/rviz/rastreator_only.rviz})
