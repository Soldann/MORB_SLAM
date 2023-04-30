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
    : hasViewer(hasViewer), poseCallback(poseCallback), cameraYaw{0}, prevRotOffset{Eigen::Matrix3f::Identity()}, gotFirstPoint{false}, initedRot{false}, mapCount{0} {
    //create SLAM and Viewer instance (Viewer if needed)
    SLAM = std::make_shared<MORB_SLAM::System>(vocab_path, settings_path, MORB_SLAM::CameraType::IMU_STEREO);

    if (hasViewer) {
        viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM, settings_path);
    }

    lastPose = Pose3D{Eigen::Vector3f(0, 0, 0), Eigen::MatrixXf::Identity(3, 3)};
}

// returns the calculated pose, once image/imu data is sent to SLAM instance
Pose SLAIV::SLAPI::sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                            const float& im_timestamp, MORB_SLAM::IMU::Point& imuMeas) {

    return sendImageAndImuData(imLeft, imRight, im_timestamp, {imuMeas});
}

Pose SLAIV::SLAPI::sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                            const float& im_timestamp, std::vector<MORB_SLAM::IMU::Point>& vImuMeas) {

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
    float currTheta = atan2(sophusPose.rotationMatrix()(1,0), sophusPose.rotationMatrix()(0,0));
    Pose3D pos{sophusPose.translation(), Eigen::Matrix3f{{cos(currTheta), -sin(currTheta), 0},
                                                    {sin(currTheta), cos(currTheta), 0},
                                                    {0,0,1}}};

    std::ofstream slamFile {"slam_pose", std::ofstream::app};
    slamFile << std::setprecision(15) << pos.translation(0) << "," <<  pos.translation(1) << "," << pos.translation(2) << std::endl;

    // when a map is first initialized, set the previous last known pose (so the map starts where the previous left off)
    // do the same process when SLAM merges the map with a previous map (it does the same thing as initialization, but instead spawns at the point in the map -- not the origin)
    
    // that is why we need to keep track of the previous map's last position & rotation AND the starting position & rotation, and transform it so
    // there is a seamless transition of poses and rotation between maps and after it merges maps

    if (!SLAM->getIsDoneVIBA() || SLAM->mpLoopCloser->loopClosed) {

        if (SLAM->mpLoopCloser->loopClosed) {
            SLAM->mpLoopCloser->loopClosed = false;

            // for debugging purposes
            std::ofstream slamFileTrans {"slam_pose_trans", std::ofstream::app};
            slamFileTrans << "loop closed here" << std::endl;
        }

        float x;
        float y;

        poseCallback(x, y, originTheta);

        originPose.translation = Eigen::Vector3f(x, y, 0);

        // rotation i set based on where the robot is before starting initing map

        gotFirstPoint = false;
        initedRot = true;

        std::cout << "pose callback: " << std::setprecision(15) << x << "," << y << "," << originTheta << std::endl;

        std::ofstream slamFileTrans {"slam_pose_trans", std::ofstream::app};
        slamFileTrans << "odom: " << std::setprecision(15) << originPose.translation(0) << "," <<  originPose.translation(1) << "," << originTheta << std::endl;

        return Pose{originPose.translation(0), originPose.translation(1), originTheta};

    } else {
        pos.translation = Eigen::Vector3f{pos.translation(0), pos.translation(1), pos.translation(2)}; // why is this needed!!??1?

        //offset by this map's global origin
        if (!gotFirstPoint) {
            mapCount++; // debugging purposes

            std::cout << "last pose theta: " << atan2(lastPose.rotation(1,0), lastPose.rotation(0,0))
                    << " origin theta: " << originTheta
                    << " camera yaw: " << cameraYaw << std::endl;

            float angle = atan2(lastPose.rotation(1,0), lastPose.rotation(0,0)) - originTheta;

            prevRotOffset =  Eigen::Matrix3f{{cos(angle), -sin(angle), 0},
                                            {sin(angle), cos(angle), 0},
                                            {0,0,1}};

            originPose.translation -= originPose.rotation * pos.translation;

            gotFirstPoint = true;

            std::cout << "origin translation: " << originPose.translation << std::endl;
            std::cout << "origin last pose theta: " << angle << std::endl;

            std::ofstream slamFile {"slam_pose", std::ofstream::app};
            slamFile << "origin: " << std::setprecision(15) << originPose.translation(0) << "," <<  originPose.translation(1) << "," << angle << std::endl;

        }

        std::cout << " curr pose theta: " << atan2(pos.rotation(1,0), pos.rotation(0,0)) << std::endl;

        Eigen::Matrix3f cameraYawOffset = Eigen::Matrix3f{{cos(-((float)M_PI)/2-cameraYaw), -sin(-((float)M_PI)/2-cameraYaw), 0},
                                                        {sin(-((float)M_PI)/2-cameraYaw), cos(-((float)M_PI)/2-cameraYaw), 0},
                                                        {0,0,1}};

        std::cout << "pos here is: " << pos.translation(0) << " , " << pos.translation(1) << std::endl;
        
        // NOTE: the rotation works fine  (it is what we expect, it is close enough to the odometry)
        // BUT the translation goes to infinity, which needs to be fixed -- see previous commit "multiple map works! " 5b8758074ef47e8db3911e2a7cb91ee7e37f1736
        // where the loop closing was not implemented, but the translation & rotation worked for the first map

        lastPose.translation =  originPose.rotation * cameraYawOffset * pos.translation + originPose.translation;

        lastPose.rotation = pos.rotation * originPose.rotation * prevRotOffset;

        std::ofstream slamFileTrans {"slam_pose_trans", std::ofstream::app};
        slamFileTrans << std::setprecision(15) << lastPose.translation(0) << "," <<  lastPose.translation(1) << "," << 
            atan2(lastPose.rotation(1,0), lastPose.rotation(0,0)) << std::endl;

        return Pose{lastPose.translation(0), lastPose.translation(1), atan2(lastPose.rotation(1,0), lastPose.rotation(0,0))};
    }
}

void SLAIV::SLAPI::setSLAMState(MORB_SLAM::Tracker::eTrackingState state) {
    SLAM->setTrackingState(state);
}
