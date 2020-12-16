# Install intelrealsense software

Now (December 2020) the only version available for **ROS2** is `intel librealsense v2.38.1`. However the last version is  `librealsense v2.40.0`. 



## Build librealsense

1. As we need a previous version of the `librealsense` library, we need to compile it.

   1. ```bash
      git clone https://github.com/IntelRealSense/librealsense
      ```

   2. ```bash
      git checkout v2.38.1   
      ```

   3. ```bash
      sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
      ```

   4. ```bash
      sudo udevadm control --reload-rules && udevadm trigger
      ```

   5. ```bash
      mkdir build && cd build
      ```

   6. If building it on arm boards:

      ```bash
      cmake ../ -DFORCE_LIBUVC:bool=true -DFORCE_RSUSB_BACKEND:bool=true -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA:bool=false -DBUILD_PYTHON_BINDINGS=bool:true
      ```

      If building it on amd boards:

      ```bash
      cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA:bool=false -DBUILD_PYTHON_BINDINGS=bool:true
      ```

      If want to use  `CUDA` set `-DBUILD_WITH_CUDA:bool=false` to `true`.

   7. ```bash
      sudo make uninstall && make clean && make -j$(nproc) && sudo make install
      ```

   8. If building it on arm boards:

      ```bash
      sudo ln -s /usr/local/lib/librealsense2.so.2.38.1 /usr/lib/aarch64-linux-gnu/librealsense2.so
      ```

      If building it on amd boards:

      ```bash
      sudo ln -s /usr/local/lib/librealsense2.so.2.38.1 /usr/lib/librealsense2.so
      ```

   9. Test it : 

      ```bash
      realsense-viewer
      ```

      

## Clone realsense-ros 

```bash
git clone https://github.com/IntelRealSense/realsense-ros -b eloquent
```



## Clone depth-to-laserscan 

```bash
git clone https://github.com/ros-perception/depthimage_to_laserscan -b foxy-devel
```



