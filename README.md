# Rastreator (Eloquent branch)
ROS2 differential wheel robot.



## Installation



### Method 1: Automatic

Open a new terminal and go to the repository root folder:

```bash
sudo chmod +x rastreator_install.sh
```

```bash
./rastreator_install.sh
```

This will install all `Ros2` and `Gazebo9` apt packages and also all `Python` dependencies.

When the installation ends, copy the following in your `~/.bashrc` file:

[Create ROS2/GAZEBO9 local variables](https://github.com/ageve/ros2_navigation/tree/rastreator/doc/installation/Local%20variables%20for%20ROS2%20and%20Gazebo.md)



### Method 2: Step by step

1. [Install ROS2 eloquent](https://github.com/iggyrrieta/rastreator/blob/eloquent/doc/installation/ROS2%20Eloquent%20installation.md)

2. [Install GAZEBO9](https://github.com/iggyrrieta/rastreator/blob/eloquent/doc/installation/Gazebo%20installation.md)

3. [Create ROS2/GAZEBO9 local variables](https://github.com/iggyrrieta/rastreator/blob/eloquent/doc/installation/Local%20variables%20for%20ROS2%20and%20Gazebo.md)

4. Install python dependencies using `pip`

   Go to repository root folder and run in a terminal:

   `pip install -r simulation_requirements.txt`
   
5. Instal the following apt dependencies:

   `sudo apt-get install v4l-utils`
   
   `sudo apt-get install ros-eloquent-navigation2`
   
   `sudo apt-get install ros-eloquent-nav2-bringup`
   
   `sudo apt-get install ros-eloquent-image-transport-plugins`
   
   `sudo apt-get install ros-eloquent-dynamixel-sdk`

   `sudo apt-get install python-rosdep2`
   
   `sudo apt install python3-sympy`



#### Install navigation2

In my case I couldn't make it work without building it from source:

```bash
# Go to <ros2 workspace folder>/src
git clone https://github.com/ros-planning/navigation2.git --branch eloquent-devel
# Go back to the root ros2 workspace folder
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
# Then
sudo rosdep init
# and
rosdep update
```



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
sudo apt-get remove ros-*
```



## Usage



### Manual control

Terminal 1 -> `ros2 launch rastreator_bringup start_manual.launch.py`

Terminal 2 -> `rviz2` (file/open Config/{path/to/rastreator_description/rviz/rastreator_only.rviz})
