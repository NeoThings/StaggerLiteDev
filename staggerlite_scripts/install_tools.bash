#! /bin/bash

#Orocos Bayesian Filtering Library
case "$(lsb_release -sc)" in
    jammy|bullseye) #22.04
        sudo apt install liborocos-bfl-dev  ;;
    focal|buster) #20.04 noetic
        sudo apt install liborocos-bfl-dev  ;;
    bionic) #18.04 melodic
        sudo apt install ros-melodic-bfl  ;;
esac

cd ..
mkdir staggerlite_tools
cd staggerlite_tools
git clone https://github.com/ros-planning/robot_pose_ekf.git