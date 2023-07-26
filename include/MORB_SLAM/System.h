/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/Tracking.h"
#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/LocalMapping.h"
#include "MORB_SLAM/LoopClosing.h"
#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/Settings.h"
#include "MORB_SLAM/Camera.hpp"
#include "MORB_SLAM/Packet.hpp"


namespace MORB_SLAM
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        std::cout << "Level: " << lev << " | " << str << std::endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const std::string &strVocFile, const std::string &strSettingsFile, const CameraType sensor);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    StereoPacket TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, double timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(), std::string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    RGBDPacket TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, double timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(), std::string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    MonoPacket TrackMonocular(const cv::Mat &im, double timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(), std::string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    virtual ~System();

    // TODO: Save/Load functions
    // SaveMap(const std::string &filename);
    // LoadMap(const std::string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    TrackingState GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

    friend Viewer;

    bool getHasMergedLocalMap();
    bool getIsDoneVIBA();

    void setTrackingState(TrackingState state);

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

private:

    void SaveAtlas(int type);
    bool LoadAtlas(int type);

    std::string CalculateCheckSum(std::string filename, int type);

    // Input sensor
    CameraType mSensor;
    std::vector<Camera_ptr> cameras;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas_ptr mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    TrackingState mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    //
    std::string mStrLoadAtlasFromFile;
    std::string mStrSaveAtlasToFile;

    std::string mStrVocabularyFilePath;

    Settings* settings_;
};
typedef std::shared_ptr<System> System_ptr;
typedef std::weak_ptr<System> System_wptr;

}// namespace ORB_SLAM
