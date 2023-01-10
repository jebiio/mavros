#! /usr/bin/env bash

set -e

mkdir -p ~/mavros_ws/src
cd ~/mavros_ws
catkin init
wstool init src

cd ~/mavros_ws
cd ~/mavros_ws/src
git clone https://github.com/jebiio/mavros.git
cd mavros
git checkout kriso_ros1
chmod +x ./pre_install.sh
./pre_install.sh

cd ~/mavros_ws/src
git clone https://github.com/jebiio/mavlink_kriso.git mavlink

cd ~/mavros_ws
catkin build
source ~/mavros_ws/devel/setup.bash

cd ~
git clone https://github.com/jebiio/kriso_ws.git
cd ~/kriso_ws
catkin build
source ~/kriso_ws/devel/setup.bash
