#! /usr/bin/env bash

set -e

source /opt/ros/noetic/setup.bash

rm -rf ~/mavros2023_ws
mkdir -p ~/mavros2023_ws/src
cd ~/mavros2023_ws
catkin init
wstool init src

cd ~/mavros2023_ws
cd ~/mavros2023_ws/src
git clone https://github.com/jebiio/mavros.git
cd mavros
git checkout kriso_ros1_2023
# git checkout kriso_ros1
# chmod +x ./pre_install.sh
# ./pre_install.sh

cd ~/mavros2023_ws/src
git clone https://jeyong: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@github.com/jebiio/mavlink_kriso.git mavlink
cd mavlink
git checkout kriso2023

cd ~/mavros2023_ws/src
git clone https://github.com/jebiio/kriso_msgs.git

cd ~/mavros2023_ws/src
git clone https://github.com/jebiio/rosserial.git
cd rosserial
git checkout kriso_noetic-devel


cd ~/mavros2023_ws
catkin build
source ~/mavros2023_ws/devel/setup.bash

# cd ~
# git clone https://github.com/jebiio/kriso_ws.git
# cd ~/kriso_ws
# catkin build
# source ~/kriso_ws/devel/setup.bash
