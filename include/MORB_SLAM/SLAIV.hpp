#pragma once
#include <string>
#include <vector>

#include "MORB_SLAM/System.h"

namespace SLAIV {
    class SLAPI { // SLAM to AIV API

        // MORB_SLAM::System_ptr SLAM; // SLAM instance, is a shared_ptr

        bool hasViewer; //only initialize the viewer shared_ptr if a viewer is required
        shared_ptr<MORB_SLAM::Viewer> viewer;

        typedef std::function<void(double& x, double& y, double& theta)> poseCallbackFunc;
        poseCallbackFunc poseCallback;

        Sophus::SE3f lastPose;
        Sophus::SE3f originPose;

        double cameraYaw;
        
        bool gotFirstPoint;
        
        public:
            MORB_SLAM::System_ptr SLAM; // SLAM instance, is a shared_ptr -- TEMPORARY FOR TESTING

            SLAPI(std::string vocab_path, std::string settings_path, bool hasViewer, poseCallbackFunc poseCallback); // initializes the tracking and local mapping threads

            void setCameraExtrin(double c) { cameraYaw = c; std::cout << "got camera yaw: " << cameraYaw << std::endl; }
            
            /**
             * to call when sending data (images, imu, ...) to SLAM, sets the lastPose
            */ 
            void sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                                 const double& im_timestamp, MORB_SLAM::IMU::Point& imuMeas);

            void sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                                 const double& im_timestamp, std::vector<MORB_SLAM::IMU::Point>& imuMeas);
            

            /**
             * Add comment here
             * @return The current state of the Tracker in SLAM
            */
            inline MORB_SLAM::Tracker::eTrackingState getSLAMState() {return SLAM->GetTrackingState();}
            
            
            /** 
             * Add comment here
             * @return A Sophus 3x3 floating point matrix of the pose
            */
            inline Sophus::SE3f getPose() { return lastPose; };

            void setSLAMState(MORB_SLAM::Tracker::eTrackingState state);


    };
}