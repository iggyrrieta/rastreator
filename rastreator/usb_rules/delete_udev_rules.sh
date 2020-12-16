#!/bin/bash

echo "deleting remap the device serial port(ttyUSBX)"
sudo rm   /etc/udev/rules.d/rplidar.rules
sudo rm   /etc/udev/rules.d/wheelmotors.rules
sudo service udev reload
sudo service udev restart
echo "finish  delete"
