# Camera Streaming

This package contains own coding for raspberry pi camera and usb camera. It also has some launch files to use `intelrealsense` cameras connected via USB.

To install `intelrealsense` libraries check [this](install_intelrealsense.md)

## Usage

**Using ROS2 (picam and usb cam)**

```bash
# Pick the one you want to launch
ros2 launch rastreator_camera picam.launch.py
ros2 launch rastreator_camera usb_cam.launch.py
```

These launch files run the camera streaming and the compression of the images.

```bash
ros2 launch rastreator_camera calibrate_cam.launch.py
```

This launch file runs the camera and the calibration module 



**Using ROS2 (intel realsense launch files connected to realsense-ros package)**

```bash
ros2 launch rastreator_camera intel_multicam.launch.py
```


**Using Direct Streaming (bash files)**

```bash
# Pick the one you want to stream
bash/intel_D435.sh
bash/picam_stream.sh
```

Using these bash files is it possible to stream the camera using your ip adress.



## Requirements

- gstreamer -> https://gstreamer.freedesktop.org/download/



v4l-utils to access the camera:

```bash
sudo apt install v4l-utils
```

To calibrate:

```bash
git clone -b ros2 https://github.com/ros-perception/image_pipeline
```

In order to make the camera work a configuration file it's needed. First time, this configuration should be any configuration but the camera name must be okay. To know the camera name:

```bash
v4l2-ctl --all
```

The name is the one at  `Driver name` attribute.

Modify the name using parameter inside file  `rastreator_camera/param/video.yaml` . You can also modify the path, the default one is the one located at `~/.ros/camera_info/<yaml_file>` 



To compress images:

```bash
sudo apt-get install ros-foxy-image-transport-plugins
```

> `image_transport` only supports raw transport by default and needs additional plugins to provide compression.



## References

https://gitlab.com/boldhearts/ros2_v4l2_camera

https://github.com/klintan/ros2_usb_camera

https://github.com/ros-perception/vision_opencv/tree/ros2

https://navigation.ros.org/tutorials/docs/camera_calibration.html

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

