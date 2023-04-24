#pragma once
#include <string>
#include <vector>

#include "MORB_SLAM/System.h"

// have to create similar struct to Sophus::SE3f because sophus throws assert failures when doing rotation math :(
struct Pose3D {
    Eigen::Vector3f translation;
    Eigen::Matrix3f rotation;
};

struct Pose {
    double x;
    double y;
    double theta;
};

namespace SLAIV {
    class SLAPI { // SLAM to AIV API

        // MORB_SLAM::System_ptr SLAM; // SLAM instance, is a shared_ptr

        bool hasViewer; //only initialize the viewer shared_ptr if a viewer is required
        shared_ptr<MORB_SLAM::Viewer> viewer;

        typedef std::function<void(double& x, double& y, double& theta)> poseCallbackFunc;
        poseCallbackFunc poseCallback;

        Pose3D lastPose;
        Pose3D originPose;

        double cameraYaw;
        
        bool gotFirstPoint;
        bool initedRot;
        
        public:
            MORB_SLAM::System_ptr SLAM; // SLAM instance, is a shared_ptr -- TEMPORARY FOR TESTING

            SLAPI(std::string vocab_path, std::string settings_path, bool hasViewer, poseCallbackFunc poseCallback); // initializes the tracking and local mapping threads

            void setCameraExtrin(double c) { cameraYaw = c; std::cout << "got camera yaw: " << cameraYaw << std::endl; }

            bool isDoneInitingMap() { return SLAM->getIsDoneVIBA(); }
            
            /**
             * to call when sending data (images, imu, ...) to SLAM, sets the lastPose
            */ 
            Pose sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                                 const double& im_timestamp, MORB_SLAM::IMU::Point& imuMeas);

            Pose sendImageAndImuData(const cv::Mat& imLeft, const cv::Mat& imRight,
                                 const double& im_timestamp, std::vector<MORB_SLAM::IMU::Point>& imuMeas);
            

            /**
             * Add comment here
             * @return The current state of the Tracker in SLAM
            */
            inline MORB_SLAM::Tracker::eTrackingState getSLAMState() {return SLAM->GetTrackingState();}
            
            void setSLAMState(MORB_SLAM::Tracker::eTrackingState state);


    };
}