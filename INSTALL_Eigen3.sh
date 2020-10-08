#! /bin/bash

cd ~

# Download EIGEN3 zip file
echo "Download eigen 3.3.8 version zip file..."
wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.zip

# INSTALL EIGEN3
echo "Install Eigen3..."
unzip eigen3.zip
cd eigen-3.3.8
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install

cd ~
rm -rf eigen3.zip eigen-3.3.8
