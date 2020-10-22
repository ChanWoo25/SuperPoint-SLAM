#! /bin/bash

# TUM dataset

############################################################
############################################################

# fr1_xyz
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM1.yaml ~/data_odometry_gray/fr1_xyz/

# fr2_xyz
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM2.yaml ~/data_odometry_gray/fr2_xyz/

# fr1_floor
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM1.yaml ~/data_odometry_gray/fr1_floor/

# fr1_desk
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM1.yaml ~/data_odometry_gray/fr1_desk/

# fr2_360_kidnap
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM2.yaml ~/data_odometry_gray/fr2_360_kidnap/

# fr2_desk
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM2.yaml ~/data_odometry_gray/fr2_desk/

# fr3_long_office
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_long_office/

############################################################
############################################################

# fr3_nstr_tex_far
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_nstr_tex_far/
# lost도 잘 일어나고 방향도 뭔가 반대

# fr3_nstr_tex_near
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_nstr_tex_near/
# corrupted size vs. prev_size      (core dumped)
# 초기 맵만 잘 만들면 어느정도는 되지만 끝가지 가기엔 무리가 있음

# fr3_str_tex_far
 Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_str_tex_far/
# 잘됨

# fr3_str_tex_near
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_str_tex_near/
# 잘 안움직임.
# free():invalid next size (fast)   (core dumped)

############################################################
############################################################

# fr2_desk_person
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM2.yaml ~/data_odometry_gray/fr2_desk_person/
# 처음에 lost되면 그 뒤로 계속 안됨
# 처음 lost 이후 간혹 reset이 되면 잘 되다가 중간에 lost

# fr3_sit_xyz
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_sit_xyz/
# 초반에만 잘 되다가 lost, 아마 카메라가 잘 안움직여서?
# corrupted size vs. prev_size      (core dumped)

# fr3_sit_halfsph
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_sit_halfsph/
# lost가 너무 잘됨
# double free or dorruption (!prev) (core dumped)

# fr3_walk_xyz
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_walk_xyz/
# lost가 너무 잘됨
# double free or corruption (out)   (core dumped)

# fr3_walk_halfsph
# Examples/Monocular/sp_mono_tum Voca/tempVoc.txt Examples/Monocular/SPTUM3.yaml ~/data_odometry_gray/fr3_walk_halfsph/
# 그냥 잘 안됨
# corrupted size vs. prev_size      (core dumped)
