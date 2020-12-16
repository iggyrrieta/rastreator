# Navigation2

Official info [here](https://navigation.ros.org/getting_started/index.html)



## Dependencies

```bash
sudo apt install ros-foxy-navigation2
```

```bash
sudo apt install ros-foxy-nav2-bringup 
```

**Cartographer**

```bash
sudo apt install ros-foxy-cartographer-ros
```

**Slam_toolbox**

```bash
sudo apt install ros-foxy-slam-toolbox
```

## Usage



**Option 1: Mapping**

1. Create map

   ```bash
   ros2 launch rastreator_bringup mapping_simulation.launch.py
   ```

   

2. Save map

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/map
   ```



**Option 2: Localization**

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=/home/iggy/ros/foxy/src/rastreator/rastreator/rastreator_navigation2/map/map.yaml

```

