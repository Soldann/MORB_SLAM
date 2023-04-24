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

    lastPose = Pose3D{Eigen::Vector3f(0, 0, 0), Eigen::MatrixXf::Identity(3, 3)};
}

// returns the calculated pose, once image/imu data is sent to SLAM instance
Pose SLAIV::SLAPI::sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                            const double& im_timestamp, MORB_SLAM::IMU::Point& imuMeas) {

    return sendImageAndImuData(imLeft, imRight, im_timestamp, {imuMeas});
}

Pose SLAIV::SLAPI::sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                            const double& im_timestamp, std::vector<MORB_SLAM::IMU::Point>& vImuMeas) {

    float imageScale = SLAM->GetImageScale();

    std::cout << "image scale: " << imageScale << std::endl;

    if(imageScale != 1.f) {
        int width = imLeft.cols * imageScale;
        int height = imLeft.rows * imageScale;
        cv::resize(imLeft, imLeft, cv::Size(width, height));
        cv::resize(imRight, imRight, cv::Size(width, height));
    }
    
    Sophus::SE3f sophusPose = SLAM->TrackStereo(imLeft, imRight, im_timestamp, vImuMeas);

    if (hasViewer) 
        viewer->update(sophusPose);

    sophusPose = sophusPose.inverse();
    Pose3D pos{sophusPose.translation(), sophusPose.rotationMatrix()};

    std::ofstream slamFile {"slam_pose", std::ofstream::app};
    slamFile << std::setprecision(15) << pos.translation(0) << "," <<  pos.translation(1) << "," << pos.translation(2) << std::endl;

    //when a map is first initialized, set the previous last known pose (so the map starts where the previous left off)
    if (!SLAM->getIsDoneVIBA()) {
        double x;
        double y;
        double theta;

        poseCallback(x, y, theta);

        originPose.translation = Eigen::Vector3f(x, y, 0);

        // rotation i set based on where the robot is before starting initing map
        if (!initedRot) {
            originPose.rotation = Eigen::Matrix3f{{cos(theta), -sin(theta), 0},
                                                    {sin(theta), cos(theta), 0},
                                                    {0,0,1}};
        }

        gotFirstPoint = false;
        initedRot = true;

        std::cout << "origin rotation: " << originPose.rotation << std::endl;
        std::cout << "pose callback: " << std::setprecision(15) << x << "," << y << "," << theta << std::endl;

        std::ofstream slamFileTrans {"slam_pose_trans", std::ofstream::app};
        slamFileTrans << std::setprecision(15) << originPose.translation(0) << "," <<  originPose.translation(1) << "," << theta << std::endl;

        std::cout << "DONE" << std::endl;

        return Pose{originPose.translation(0), originPose.translation(1), theta};

    } else {
        //weird coordinate transforms
        initedRot = false;

        pos.translation = Eigen::Vector3f(pos.translation(0), pos.translation(1), pos.translation(2));
        Eigen::Matrix3f cameraYawOffset = Eigen::Matrix3f{{cos(-M_PI/2-cameraYaw), -sin(-M_PI/2-cameraYaw), 0},
                                                    {sin(-M_PI/2-cameraYaw), cos(-M_PI/2-cameraYaw), 0},
                                                    {0,0,1}};
        //offset by this map's global origin
        if (!gotFirstPoint) {
            originPose.translation -= originPose.rotation * cameraYawOffset * pos.translation;
            gotFirstPoint = true;

            std::cout << "origin translation: " << originPose.translation << std::endl;

            std::ofstream slamFile {"slam_pose", std::ofstream::app};
            slamFile << "origin: " << std::setprecision(15) << originPose.translation(0) << "," <<  originPose.translation(1) << "," << originPose.translation(2) << std::endl;

        }
        
        lastPose.translation =  originPose.rotation * cameraYawOffset * pos.translation +  originPose.translation;
        lastPose.rotation = originPose.rotation * pos.rotation;

        std::ofstream slamFileTrans {"slam_pose_trans", std::ofstream::app};
        slamFileTrans << std::setprecision(15) << lastPose.translation(0) << "," <<  lastPose.translation(1) << "," << 
            atan2(lastPose.rotation(1,0), lastPose.rotation(0,0)) << std::endl;

        return Pose{lastPose.translation(0), lastPose.translation(1), atan2(lastPose.rotation(1,0), lastPose.rotation(0,0))};
    }
}

void SLAIV::SLAPI::setSLAMState(MORB_SLAM::Tracker::eTrackingState state) {
    SLAM->setTrackingState(state);
}
