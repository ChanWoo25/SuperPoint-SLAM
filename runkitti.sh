#! /bin/bash

#./Examples/Monocular/sp_mono_kitti ./SPVoc1.txt ./Examples/Monocular/SPKITTI00-02.yaml ~/Dataset/dataset/sequences/00/

#./Examples/Monocular/sp_mono_kitti ./SPVoc0_Iter500_Img7226.txt ./Examples/Monocular/SPKITTI00-02.yaml ~/Dataset/dataset/sequences/00/


# Kitti Sequence [00]
# ./Examples/Monocular/sp_mono_kitti Voca/tempVoc.txt ./Examples/Monocular/SPKITTI00-02.yaml ~/data_odometry_gray/dataset/sequences/00

# Kitti Sequence [05]
Examples/Monocular/sp_mono_kitti Voca/tempVoc.txt Examples/Monocular/SPKITTI04-12.yaml ~/data_odometry_gray/dataset/sequences/05 0

Examples/Monocular/sp_mono_kitti Voca/tempVoc.txt Examples/Monocular/SPKITTI04-12.yaml ~/data_odometry_gray/dataset/sequences/05 1

Examples/Monocular/sp_mono_kitti Voca/tempVoc.txt Examples/Monocular/SPKITTI04-12.yaml ~/data_odometry_gray/dataset/sequences/05 2

Examples/Monocular/sp_mono_kitti Voca/tempVoc.txt Examples/Monocular/SPKITTI04-12.yaml ~/data_odometry_gray/dataset/sequences/05 3

Examples/Monocular/sp_mono_kitti Voca/tempVoc.txt Examples/Monocular/SPKITTI04-12.yaml ~/data_odometry_gray/dataset/sequences/05 4

# 00 : 맵이 커지다가 lost
# 01 : 건물 없는 고속도로. 금방 lost
# 02 : 건물 없고 나무만 나오는 장면에서 lost
# 03 : 건물 없고 나무만 나오는 장면에서 lost
# 04 : 작은 데이터셋. 길이 그냥 직진만 함
# 05 : best data
# 06 : 길게 하나의 loop, 중간에 lost
# 07 : 적당히 큰 하나의 loop, 중간에 큰 트럭이 앞에 지나가면서 lost
# 08 : 큰 데이터셋. 잘 가다가 나무가 많고 특징이 별로 없는 도로에서 lost
# 09 : 건물이 별로 없는 도로에서 lost
# 10 : 골목에서 좀 빨리 달리더니 lost
# 11 : 초기화 자체가 안됨
# 12 : 주변에 건물이 있어서 금방 lost

