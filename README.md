# SuperPoint-SLAM
---

**Authors:** [Chanwoo Lee](https://github.com/ChanWoo25)

**Note** This is my personal project. SuperPoint-SLAM is a project implemented in order to analyze the slightly modified ORB-SLAM. ORB-SLAM's license is as below:

# 1. License

**ORB-SLAM2 Authors** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. Prerequisites
We have tested the library in **16.04**, but it should be easy to compile in other platforms. To build **SuperPoint-SLAM**, you need a GPU that supports CUDA 10.2 or higher. Of course, you can use a lower CUDA version by modifying the LibTorch code accordingly. However, I cannot guarantee normal operation. Also, To some extent, powerful cpu is required for real-time performance and more stable and accurate results of SuperPointSLAM.

Ok, so let's first look at *the build requirements*. You can conveniently install the necessary libraries using the Basg Scripts we wrote.
However, you can also install it yourself after looking at the official website by looking at the library list described below. If you install it yourself, **keep in mind that** we put all the important libraries in "/usr/local".

To use our scripts. Note that the order of execution matters because Eigen3 can be used other libraries' build.
```
git clone https://github.com/ChanWoo25/SuperPoint-SLAM.git
cd SuperPoint-SLAM
chmod +x INSTALL_*
bash INSTALL_Eigen3.sh
bash INSTALL_OpenCV.sh
bash INSTALL_LibTorch.sh
bash INSTALL_Pangolin.sh
```

## 1. C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## 2. Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **(Tested with Eigen 3.3.8 (At least 3.1.0 required))**.

## 3. OpenCV
We used the latest version of [OpenCV](http://opencv.org) for stable operation. **(Tested with OpenCV 3.2.0 and 3.4.11)**

## 4. LibTorch 
We used the 1.6.0 version of [LibTorch](https://pytorch.org/) for SuperPoint Implementation.

## 5. Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## 6. DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.


# 3. Building SuperPoint-SLAM library and examples

TODO...

# 4. Monocular Examples

## TUM Dataset

See runtum.sh

## KITTI Dataset  

See runkitti.sh

# 4. Download Link for SuperPoint-SLAM Vocabulary
SOON

