#pragma once

#include <iostream>
#include <array>

namespace MORB_SLAM{


class TrackingState{
  static size_t size;
  size_t id;
  TrackingState(const char *toString):id{++size}, toString{toString} {}
public:
  const char *toString;

  static const TrackingState SYSTEM_NOT_READY;
  static const TrackingState NO_IMAGES_YET;
  static const TrackingState NOT_INITIALIZED;
  static const TrackingState OK;
  static const TrackingState RECENTLY_LOST;
  static const TrackingState LOST;
  static const TrackingState OK_KLT;
  static const std::array<TrackingState, 7> STATES;

  bool operator==(const TrackingState &other) const { return id == other.id; }
  bool operator!=(const TrackingState& other) const { return !(*this == other); }
};
inline std::ostream &operator<<(std::ostream &os, const TrackingState &t){
    os << t.toString;
    return os;
}



class CameraType{
  static size_t size;
  size_t id;
  CameraType(bool isInertial, bool hasMulticam, const char *toString):id{++size}, _isInertial{isInertial}, _hasMulticam{hasMulticam}, toString{toString} {}
  bool _isInertial;
  bool _hasMulticam;
public:
  const char *toString;

  static const CameraType MONOCULAR;
  static const CameraType STEREO;
  static const CameraType RGBD;
  static const CameraType IMU_MONOCULAR;
  static const CameraType IMU_STEREO;
  static const CameraType IMU_RGBD;
  static const std::array<CameraType, 6> TYPES;

  bool isInertial() const { return _isInertial; }
  bool hasMulticam() const { return _hasMulticam; }
  bool operator==(const CameraType &other) const { return id == other.id; }
  bool operator!=(const CameraType& other) const { return !(*this == other); }
};
inline std::ostream &operator<<(std::ostream &os, const CameraType &t){
    os << t.toString;
    return os;
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