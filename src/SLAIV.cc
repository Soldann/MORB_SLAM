/**
 * This file is part of MORB-SLAM3
 * 
 * Add additional stuff here
*/

#include "MORB_SLAM/SLAIV.hpp"
#include "MORB_SLAM/Viewer.h"

#include <fstream>
#include <iomanip>
#include <math.h>

SLAIV::SLAPI::SLAPI(std::string vocab_path, std::string settings_path, bool hasViewer, poseCallbackFunc poseCallback) 
    : hasViewer(hasViewer), poseCallback(poseCallback), gotFirstPoint{false}, cameraYaw{0} {
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
                            const double& im_timestamp, std::vector<MORB_SLAM::IMU::Point>& vImuMeas) {

    float imageScale = SLAM->GetImageScale();

    std::cout << "image scale: " << imageScale << std::endl;

    if(imageScale != 1.f) {
        int width = imLeft.cols * imageScale;
        int height = imLeft.rows * imageScale;
        cv::resize(imLeft, imLeft, cv::Size(width, height));
        cv::resize(imRight, imRight, cv::Size(width, height));
    }
    
    Sophus::SE3f pos = SLAM->TrackStereo(imLeft, imRight, im_timestamp, vImuMeas);

    if (hasViewer) 
        viewer->update(pos);

    pos = pos.inverse();

    //when a map is first initialized, set the previous last known pose (so the map starts where the previous left off)
    if (!SLAM->getIsDoneVIBA()) {
        double x;
        double y;
        double theta;

        poseCallback(x, y, theta);

        std::cout << "theta received: " << theta << std::endl;

        theta -= cameraYaw;

        originPose.translation() = Eigen::Vector3f(x, y, 0);
        originPose.setRotationMatrix(Eigen::Matrix3f{{cos(theta), -sin(theta), 0},
                                                    {sin(theta), cos(theta), 0},
                                                    {0,0,1}});

        gotFirstPoint = false;

        std::cout << "origin rotation: " << originPose.rotationMatrix() << std::endl;
        std::cout << "pose callback: " << std::setprecision(15) << x << "," << y << "," << theta << std::endl;

    } else {
        //weird coordinate transforms
        pos.translation() = Eigen::Vector3f(-pos.translation()(1), -pos.translation()(0), pos.translation()(2));
        //offset by this map's global origin
        if (!gotFirstPoint) {
            originPose.translation() -= originPose.rotationMatrix() * pos.translation();
            gotFirstPoint = true;

            std::cout << "origin translation: " << originPose.translation() << std::endl;

            std::ofstream slamFile {"slam_pose", std::ofstream::app};
            slamFile << "origin: " << std::setprecision(15) << originPose.translation()(0) << "," <<  originPose.translation()(1) << "," << originPose.translation()(2) << std::endl;

        }
        
        lastPose.translation() =  originPose.rotationMatrix() * pos.translation() +  originPose.translation();
        lastPose.setRotationMatrix(pos.rotationMatrix() * originPose.rotationMatrix());
    }

    std::ofstream slamFile {"slam_pose", std::ofstream::app};
    slamFile << std::setprecision(15) << pos.translation()(0) << "," <<  pos.translation()(1) << "," << pos.translation()(2) << std::endl;

    std::ofstream slamFileTrans {"slam_pose_trans", std::ofstream::app};
    slamFileTrans << std::setprecision(15) << lastPose.translation()(0) << "," <<  lastPose.translation()(1) << "," << lastPose.translation()(2) << std::endl;

}

void SLAIV::SLAPI::setSLAMState(MORB_SLAM::Tracker::eTrackingState state) {
    SLAM->setTrackingState(state);
}
