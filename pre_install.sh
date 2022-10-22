#! /usr/bin/env bash

sudo apt-get install python3-catkin-tools
sudo apt-get install geographiclib-tools
sudo apt-get install libgeographic-dev
sudo ./mavros/script/install_geographiclib_datasets.sh
sudo apt install ros-noetic-geographic-msgs -y
