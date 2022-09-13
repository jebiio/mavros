#! /usr/bin/env bash

rm -rf ~/catkin_ws
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

cd ~/catkin_ws
cd ~/catkin_ws/src
git clone https://github.com/jebiio/mavros.git
cd mavros 
git checkout kriso_ros1

cd ~/catkin_ws/src
git clone https://github.com/jebiio/mavlink_kriso.git mavlink

cd ~/catkin_ws/

catkin build

source devel/setup.bash

