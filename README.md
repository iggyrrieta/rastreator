# Rastreator (Simulation branch)
ROS2 differential wheel robot.



## Installation

Clone repository and checkout branch `simulation`

```bash
git clone -b simulation https://github.com/iggyrrieta/rastreator
```



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

[Create ROS2/GAZEBO9 local variables](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/Local%20variables%20for%20ROS2%20and%20Gazebo.md)




### Method 2: Step by step

1. [Install ROS2 eloquent](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/ROS2%20Eloquent%20installation.md)

2. [Install GAZEBO9](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/Gazebo%20installation.md)

3. [Create ROS2/GAZEBO9 local variables](https://github.com/iggyrrieta/rastreator/blob/master/doc/installation/Local%20variables%20for%20ROS2%20and%20Gazebo.md)

4. Install python dependencies using `pip`

   Go to repository root folder and run in a terminal:

   `pip install -r simulation_requirements.txt`
   
5. Install `sympy`:
   
   `sudo apt install python3-sympy`



## Build

To build all packages needed for the simulation tests (if we are using branch `simulation`):

```bash
colcon build
```

Source the compilation if you want to test a package using same terminal:

```bash
source install/local_setup.bash
```



## Usage

### Simulation: TEST EKF

Terminal 1:

```bash
ros2 launch rastreator_simulation ekf_sim.launch.py 
```

A `Gazebo` and `rqt_plot` window should pop up.

More info [here](rastreator_simulation)

