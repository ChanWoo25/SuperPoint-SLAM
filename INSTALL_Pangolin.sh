#! /bin/bash

git clone https://github.com/stevenlovegrove/Pangolin.git
sudo apt install -y libgl1-mesa-dev libglew-dev

sudo mv Pangolin/ /usr/local
cd /usr/local/Pangolin
sudo mkdir build
cd build
sudo cmake ..
sudo cmake --build .