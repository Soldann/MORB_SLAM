echo "Building ROS nodes"

cd Examples/ROS/MORB_SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
