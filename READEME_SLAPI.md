## SLAM-AIV API
This API will be used to integrate SLAM into the AIV code. `SLAIV.cc` essentially wraps `System.cc` of the ORB-SLAM3 library. The AIV code will interact with the ORB-SLAM3 through the functions in `SLAIV.hpp`. 

# AIV -> SLAM
1. IMU values processed and synchronized (angular velocity and acceleration)
2. L & R camera images as two seperate images

# SLAM -> AIV
1. pose (as SE3f, which includes rotation matrix and translation vector pose coordinate frame to absolute origin)
2. tracking state (to track whether SLAM is currently lost)
3. Event handler for when the local mapped merges to inactive map (LoopClosing::MergeLocal2() function)

