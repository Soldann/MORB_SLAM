#include "MORB_SLAM/ImprovedTypes.hpp"

namespace MORB_SLAM{
size_t TrackingState::size(0);
const TrackingState TrackingState::SYSTEM_NOT_READY("SYSTEM_NOT_READY");
const TrackingState TrackingState::NO_IMAGES_YET("NO_IMAGES_YET");
const TrackingState TrackingState::NOT_INITIALIZED("NOT_INITIALIZED");
const TrackingState TrackingState::OK("OK");
const TrackingState TrackingState::RECENTLY_LOST("RECENTLY_LOST");
const TrackingState TrackingState::LOST("LOST");
const TrackingState TrackingState::OK_KLT("OK_KLT");
const std::array<TrackingState, 7> TrackingState::STATES{SYSTEM_NOT_READY, NO_IMAGES_YET, NOT_INITIALIZED, OK, RECENTLY_LOST, LOST, OK_KLT};
size_t CameraType::size(0);
const CameraType CameraType::MONOCULAR(false, false, "MONOCULAR");
const CameraType CameraType::STEREO(false, true, "STEREO");
const CameraType CameraType::RGBD(false, true, "RGBD");
const CameraType CameraType::IMU_MONOCULAR(true, false, "IMU_MONOCULAR");
const CameraType CameraType::IMU_STEREO(true, true, "IMU_STEREO");
const CameraType CameraType::IMU_RGBD(true, true, "IMU_RGBD");
const std::array<CameraType, 6> CameraType::TYPES{MONOCULAR, STEREO, RGBD, IMU_MONOCULAR, IMU_STEREO, IMU_RGBD};
}