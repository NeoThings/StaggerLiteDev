#! /bin/bash

cd ~

# case "$(lsb_release -sc)" in
#     focal|buster) #20.04
#         pip3 uninstall jinja2 markupsafe
#         pip3 install markupsafe==2.0.1
# esac

# Install the required libraries that are available as debs.
sudo apt-get update
sudo apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libceres-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    python3-sphinx \
    stow

# Install Protocol Buffers and Abseil if available.
# No need to build it ourselves.
case "$(lsb_release -sc)" in
    jammy|bullseye) #22.04
        sudo apt-get install -y libgmock-dev protobuf-compiler libabsl-dev ;;
    focal|buster) #20.04
        sudo apt-get install -y libgmock-dev protobuf-compiler ;;
    bionic) #18.04
        ;;
esac

sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

mkdir cartographer_ws
cd cartographer_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

src/cartographer/scripts/install_abseil.sh
catkin_make_isolated --install --use-ninja

echo "source ~/cartographer_ws/install_isolated/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc