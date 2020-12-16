# ROS2 Foxy installation

Install ROS2 Foxy (Debian packages):

References : [ROS2 foxy installation](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

1. Setup Local

   ```bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. Setup Sources

   ```bash
   sudo apt update && sudo apt install curl gnupg2 lsb-release
   ```
   ```bash
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

   ```bash
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   ```

3. Install ROS2 packages

   ```bash
   sudo apt update
   ```

   ```bash
   sudo apt install ros-foxy-desktop
   ```

4. Install Colcon

   ```bash
   sudo apt install python3-colcon-common-extensions
   ```