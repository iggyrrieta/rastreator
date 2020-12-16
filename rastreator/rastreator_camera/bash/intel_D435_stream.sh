#!/bin/sh

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

# Tested on Jetson Nano using Intelrealsense D435
# /dev/video2 = black/white image /dev/video3/ = colored image
gst-launch-1.0 \
 v4l2src device=/dev/video2 \
 ! 'video/x-raw,width=640,height=480,framerate=30/1' \
 ! videoconvert ! omxh264enc ! rtph264pay ! gdppay \
 ! tcpserversink host=192.168.1.138 port=5000 sync=false