#pragma once

#include <memory>
#include <utility>
#include <unordered_map>

namespace MORB_SLAM{

class System;
class Atlas;
class Tracking;
typedef std::shared_ptr<System> System_ptr;
typedef std::weak_ptr<System> System_wptr;
typedef std::shared_ptr<Atlas> Atlas_ptr;
typedef std::weak_ptr<Atlas> Atlas_wptr;
typedef std::shared_ptr<Tracking> Tracking_ptr;
typedef std::weak_ptr<Tracking> Tracking_wptr;

typedef std::pair<int, int> IntPair;
template<typename KEY, typename VALUE> using umap = std::unordered_map<KEY,VALUE>;



  namespace Tracker{
    // Tracking states
    enum eTrackingState {
        SYSTEM_NOT_READY = -1,
        NO_IMAGES_YET = 0,
        NOT_INITIALIZED = 1,
        OK = 2,
        RECENTLY_LOST = 3,
        LOST = 4,
        OK_KLT = 5
    };
  }

  namespace CameraType{
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    inline bool isInertial(eSensor sensor) { return sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD; }
    inline bool hasMulticam(eSensor sensor) { return sensor == STEREO || sensor == RGBD || sensor == IMU_STEREO || sensor == IMU_RGBD; }
  }

  namespace ImuInitializater{
    enum ImuInitType{ //enum values are used for math stuff -- DO NOT CHANGE
      MONOCULAR_INIT_A=10000000000,
      STEREO_INIT_A=100000,
      VIBA1_A=100000, // VIBA = visual-inertial bundle adjustment
      VIBA2_A=0,
      DEFAULT_A=100,
      MONOCULAR_INIT_G=100,
      STEREO_INIT_G=100,
      VIBA1_G=1,
      VIBA2_G=0,
      DEFAULT_G=1000000,
    };
  }
}