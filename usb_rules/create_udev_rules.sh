#!/bin/bash

echo "remapping the device serial port(ttyUSB*) with permissions to work"
sudo cp rplidar.rules  /etc/udev/rules.d
sudo cp wheelmotors.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish "
