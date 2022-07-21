# MORB-SLAM
Details of changes between the different versions.

### V1.0.1, 21st July 2022

- Convert compilation into a shared library format so MORBSLAM can be imported into other robotics projects
- Uodated codebase to C++ 17
- Fixed the code and removed warnings due to deprecated or unused variables
- Fixed segmentation faults when saving maps
- Fixed segmentation faults when loading maps
- Fixed segmentation faults in the Stereo-Inertial example
- Fixed segmentation faults when shutting down
- Fixed segmentation faults in EdgePriorPoseImu
- Fixed deadlocks when shutting down
- Fixed deadlocks when resetting the active map
- Added IR projector to the Stereo-Inertial example
- Made minor optimizations

### V1.0, 22th December 2021

- OpenCV static matrices changed to Eigen matrices. The average code speed-up is 16% in tracking and 19% in mapping, w.r.t. times reported in the ORB-SLAM3 paper.

- New calibration file format, see file Calibration_Tutorial. Added options for stereo rectification and image resizing.

- Added load/save map functionalities.

- Added examples of live SLAM using Intel Realsense cameras.

- Fixed several bugs.

### V0.4: Beta version, 21st April 2021

- Changed OpenCV dynamic matrices to static matrices to speed up the code.

- Capability to measure running time of the system threads.

- Compatibility with OpenCV 4.0 (Requires at least OpenCV 3.0). 

- Fixed minor bugs.


### V0.3: Beta version, 4th Sep 2020

- RGB-D compatibility: the RGB-D examples have been adapted to the new version.

- Kitti and TUM dataset compatibility: these examples have been adapted to the new version.

- ROS compatibility: updated the old references in the code to work with this version.

- Config file parser: the YAML file contains the session configuration, a wrong parametrization may break the execution without any information to solve it. This version parses the file to read all the fields and give a proper answer if one of the fields have been wrongly deffined or does not exist.

- Fixed minor bugs.


### V0.2: Beta version, 7th Aug 2020
Initial release. It has these capabilities:

- Multiple-Map capabilities: it is able to handle multiple maps in the same session and merge them when a common area is detected with a seamless fussion.

- Inertial sensor: the IMU initialization takes 2 seconds to achieve a scale error less than 5\% and it is reffined in the next 10 seconds until it is around 1\%. Inertial measures are integrated at frame rate to estimate the scale, gravity and velocity in order to improve the visual features detection and make the system robust to temporal occlusions.

- Fisheye cameras: cameras with wide-angle and fisheye lenses are now fully supported in monocular and stereo. 


