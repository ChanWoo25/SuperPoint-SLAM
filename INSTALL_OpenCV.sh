#! /bin/bash

cd ~

# Download OpenCV zip files.
echo "Download 3.4.11 version OpenCV and OpenCV Contrib..."
wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.11.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.11.zip

# INSTALL OpenCV
echo "Install OpenCV with some options..."
sudo unzip opencv.zip -d /usr/local
sudo unzip opencv_contrib.zip -d /usr/local
sudo apt install git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

cd /usr/local/opencv-3.4.11
sudo mkdir build
cd build

sudo cmake -DBUILD_opencv_world -DCMAKE_CONFIGURATION_TYPE=Release -DOPENCV_ENABLE_NONFREE -DOPENCV_EXTRA_MODULES_PATH=/usr/local/opencv_contrib-3.4.11/modules -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make -j$(nproc)
sudo make install

# Remove .zip files.
cd ~
rm -rf opencv.zip opencv_contrib.zip 
