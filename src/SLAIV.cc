/**
 * This file is part of MORB-SLAM3
 * 
 * Add additional stuff here
*/

#include "MORB_SLAM/SLAIV.hpp"
#include "MORB_SLAM/Viewer.h"

SLAIV::SLAPI::SLAPI(std::string vocab_path, std::string settings_path, bool hasViewer) 
    : hasViewer(hasViewer) {
    //create SLAM and Viewer instance (Viewer if needed)
    SLAM = std::make_shared<MORB_SLAM::System>(vocab_path, settings_path, MORB_SLAM::CameraType::IMU_STEREO);

    if (hasViewer) {
        viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM, settings_path);
    }

    lastPose.setRotationMatrix(Eigen::MatrixXf::Identity(3, 3));
    lastPose.translation() = Eigen::Vector3f(0, 0, 0);
}

// returns the calculated pose, once image/imu data is sent to SLAM instance
void SLAIV::SLAPI::sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                            const double& im_timestamp, MORB_SLAM::IMU::Point& imuMeas) {

    sendImageAndImuData(imLeft, imRight, im_timestamp, {imuMeas});
}

void SLAIV::SLAPI::sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                            const double& im_timestamp, std::vector<MORB_SLAM::IMU::Point>& imuMeas) {

    std::vector<MORB_SLAM::IMU::Point> vImuMeas = {imuMeas};
    Sophus::SE3f pos = SLAM->TrackStereo(imLeft, imRight, im_timestamp, vImuMeas);

    if (hasViewer) {
        viewer->update(pos);
    }

    lastPose = pos;
}

bool SLAIV::SLAPI::getHasMergedLocalMap() {
    bool temp = SLAM->getHasMergedLocalMap();
    SLAM->setHasMergedLocalMap(false);
    return temp;
}

bool SLAIV::SLAPI::getHasInitializedNewMap() {
    bool temp = SLAM->getHasInitializedNewMap();
    SLAM->setHasInitializedNewMap(false);
    return temp;
}

void SLAIV::SLAPI::setSLAMState(MORB_SLAM::Tracker::eTrackingState state) {
    SLAM->setTrackingState(state);
}
