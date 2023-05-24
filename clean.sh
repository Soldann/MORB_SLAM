#!/bin/bash

# https://stackoverflow.com/questions/24112727/relative-paths-based-on-file-location-instead-of-current-working-directory
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path" # change directories so working directory is where the script is

rm Vocabulary/ORBvoc.txt  2> /dev/null
rm -r build  2> /dev/null
rm -r bin 2> /dev/null

# Clean executable examples
rm Examples/RGB-D/rgbd_tum \
    Examples/RGB-D/rgbd_realsense_D435i \
    Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i \
    Examples/Monocular/mono_realsense_D435i \
    Examples/Monocular/mono_euroc \
    Examples/Monocular/mono_kitti \
    Examples/Monocular/mono_tum \
    Examples/Monocular/mono_tum_vi \
    Examples/Monocular/mono_realsense_t265 \
    Examples/Stereo/stereo_euroc \
    Examples/Stereo/stereo_kitti \
    Examples/Stereo/stereo_tum_vi \
    Examples/Stereo/stereo_realsense_t265 \
    Examples/Stereo/stereo_realsense_D435i \
    Examples/Monocular-Inertial/mono_inertial_euroc \
    Examples/Monocular-Inertial/mono_inertial_tum_vi \
    Examples/Monocular-Inertial/mono_inertial_realsense_t265 \
    Examples/Monocular-Inertial/mono_inertial_realsense_D435i \
    Examples/Monocular-Inertial/mono_inertial_realsense_D435i_2 \
    Examples/Stereo-Inertial/stereo_inertial_euroc \
    Examples/Stereo-Inertial/stereo_inertial_tum_vi \
    Examples/Stereo-Inertial/stereo_inertial_realsense_t265 \
    Examples/Stereo-Inertial/stereo_inertial_realsense_D435i \
    Examples/ROS/MORB_SLAM/Mono \
    Examples/ROS/MORB_SLAM/Mono_Inertial \
    Examples/ROS/MORB_SLAM/MonoAR \
    Examples/ROS/MORB_SLAM/RGBD \
    Examples/ROS/MORB_SLAM/Stereo \
    Examples/ROS/MORB_SLAM/Stereo_Inertial \
    Examples/Calibration/recorder_realsense_D435i \
    Examples/Calibration/recorder_realsense_T265 \
    Examples/Tests/viewer_dataset \
    Examples/Tests/sophus_test \
    2> /dev/null

rm -r Examples/ROS/MORB_SLAM/build 2> /dev/null

echo cleaning complete
