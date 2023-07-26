/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <unordered_set>
#include <fstream>

#include "MORB_SLAM/Atlas.h"
#include "MORB_SLAM/Frame.h"
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/ImuTypes.h"
#include "MORB_SLAM/KeyFrameDatabase.h"
#include "MORB_SLAM/LocalMapping.h"
#include "MORB_SLAM/LoopClosing.h"
#include "MORB_SLAM/ORBVocabulary.h"
#include "MORB_SLAM/ORBextractor.h"
#include "MORB_SLAM/Settings.h"
#include "MORB_SLAM/System.h"
#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/Camera.hpp"
#include "MORB_SLAM/Packet.hpp"

namespace MORB_SLAM {

class Tracking {
 public:
  
  Tracking(System* pSys, ORBVocabulary* pVoc,
           const Atlas_ptr &pAtlas, KeyFrameDatabase* pKFDB,
           const std::string& strSettingPath, const CameraType sensor, Settings* settings);

  ~Tracking();

  // Parse the config file
  bool ParseCamParamFile(cv::FileStorage& fSettings);
  bool ParseORBParamFile(cv::FileStorage& fSettings);
  bool ParseIMUParamFile(cv::FileStorage& fSettings);

  // Preprocess the input and call Track(). Extract features and performs stereo
  // matching.
  StereoPacket GrabImageStereo(const cv::Mat& imRectLeft,
                               const cv::Mat& imRectRight,
                               const double& timestamp, const std::string &filename,
                               const Camera_ptr &cam);
  RGBDPacket GrabImageRGBD(const cv::Mat& imRGB, const cv::Mat& imD,
                             const double& timestamp, const std::string &filename,
                             const Camera_ptr &cam);
  MonoPacket GrabImageMonocular(const cv::Mat& im, const double& timestamp,
                                  const std::string &filename, const Camera_ptr &cam);

  void GrabImuData(const std::vector<IMU::Point>& imuMeasurements);

  void SetLocalMapper(LocalMapping* pLocalMapper);
  void SetLoopClosing(LoopClosing* pLoopClosing);

  // Load new settings
  // The focal lenght should be similar or scale prediction will fail when
  // projecting points
  void ChangeCalibration(const std::string& strSettingPath);

  // Use this function if you have deactivated local mapping and you only want
  // to localize the camera.
  void InformOnlyTracking(const bool& flag);

  void UpdateFrameIMU(const float s, const IMU::Bias& b, KeyFrame* pCurrentKeyFrame);
  KeyFrame* GetLastKeyFrame() { return mpLastKeyFrame; }

  void CreateMapInAtlas();
  // std::mutex mMutexTracks;

  //--
  int GetMatchesInliers();

  float GetImageScale();

#ifdef REGISTER_LOOP
  void RequestStop();
  bool isStopped();
  void Release();
  bool stopRequested();
#endif

 public:

  TrackingState mState;
  TrackingState mLastProcessedState;

  // Input sensor
  CameraType mSensor;

  // Current Frame
  Frame mCurrentFrame;
  Frame mLastFrame;

  // Initialization Variables (Monocular)
  std::vector<int> mvIniLastMatches;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

  // Lists used to recover the full camera trajectory at the end of the
  // execution. Basically we store the reference keyframe for each frame and its
  // relative transformation
protected:
  std::list<Sophus::SE3f> mlRelativeFramePoses;
  std::list<KeyFrame*> mlpReferences;
  std::list<bool> mlbLost;
public:

  // True if local mapping is deactivated and we are performing only localization
  bool mbOnlyTracking;

  void Reset(bool bLocMap = false);
  void ResetActiveMap(bool bLocMap = false);

 protected:
  bool mFastInit = false;
 public:

#ifdef REGISTER_TIMES
  void LocalMapStats2File();
  void TrackStats2File();
  void PrintTimeStats();

  std::vector<double> vdRectStereo_ms;
  std::vector<double> vdResizeImage_ms;
  std::vector<double> vdORBExtract_ms;
  std::vector<double> vdStereoMatch_ms;
  std::vector<double> vdIMUInteg_ms;
  std::vector<double> vdPosePred_ms;
  std::vector<double> vdLMTrack_ms;
  std::vector<double> vdNewKF_ms;
  std::vector<double> vdTrackTotal_ms;
#endif

 protected:
  // Main tracking function. It is independent of the input sensor.
  void Track();

  // Map initialization for stereo and RGB-D
  void StereoInitialization();

  // Map initialization for monocular
  void MonocularInitialization();
  // void CreateNewMapPoints();
  void CreateInitialMapMonocular();

  void CheckReplacedInLastFrame();
  bool TrackReferenceKeyFrame();
  void UpdateLastFrame();
  bool TrackWithMotionModel();
  bool PredictStateIMU();

  bool Relocalization();

  void UpdateLocalMap();
  void UpdateLocalPoints();
  void UpdateLocalKeyFrames();

  bool TrackLocalMap();
  void SearchLocalPoints();

  bool NeedNewKeyFrame();
  void CreateNewKeyFrame();

  // Perform preintegration from last frame
  void PreintegrateIMU();

  // Reset IMU biases and compute frame velocity
  void ResetFrameIMU();

  bool mbMapUpdated;

  // Imu preintegration from last frame
  IMU::Preintegrated* mpImuPreintegratedFromLastKF;

  // Queue of IMU measurements between frames
  std::list<IMU::Point> mlQueueImuData;

  // Vector of IMU measurements from previous to current frame (to be filled by
  // PreintegrateIMU)
  std::mutex mMutexImuQueue;

  // Imu calibration parameters
  std::shared_ptr<IMU::Calib> mpImuCalib;

  // Last Bias Estimation (at keyframe creation)
  IMU::Bias mLastBias;

  // In case of performing only localization, this flag is true when there are
  // no matches to points in the map. Still tracking will continue if there are
  // enough matches with temporal points. In that case we are doing visual
  // odometry. The system will try to do relocalization to recover "zero-drift"
  // localization to the map.
  bool notEnoughMatchPoints_trackOnlyMode;

  // Other Thread Pointers
  LocalMapping* mpLocalMapper;
  LoopClosing* mpLoopClosing;

  // ORB
  std::shared_ptr<ORBextractor> mpORBextractorLeft;
  std::shared_ptr<ORBextractor> mpORBextractorRight;
  std::shared_ptr<ORBextractor> mpIniORBextractor;

  // BoW
  ORBVocabulary* mpORBVocabulary;
  KeyFrameDatabase* mpKeyFrameDB;

  // Initalization (only for monocular)
  bool mbReadyToInitialize;
  bool mbSetInit;

  // Local Map
  KeyFrame* mpReferenceKF;
  std::vector<KeyFrame*> mvpLocalKeyFrames;
  std::vector<MapPoint*> mvpLocalMapPoints;

  // System
  System* mpSystem;

  // Atlas
  Atlas_ptr mpAtlas;

  // Calibration matrix
  cv::Mat mK;
  Eigen::Matrix3f mK_;
  cv::Mat mDistCoef;
  float mbf;
  float mImageScale;

  float mImuFreq;
  double mImuPer;
  bool mInsertKFsLost;

  // New KeyFrame rules (according to fps)
  int mMinFrames;
  int mMaxFrames;

  int mnFirstImuFrameId;
  int mnFramesToResetIMU;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two
  // keyframes.
  float mThDepth;

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are
  // scaled.
  float mDepthMapFactor;

  // Current matches in frame
  int mnMatchesInliers;

  // Last Frame, KeyFrame and Relocalisation Info
  KeyFrame* mpLastKeyFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;
  double mTimeStampLost;
  double time_recently_lost;

  unsigned int mnFirstFrameId;
  unsigned int mnInitialFrameId;
  unsigned int mnLastInitFrameId;

  bool mbCreatedMap;

  // Motion Model
  bool imuMotionModelPrepedAfterRecentlyLostTracking{false};
  Sophus::SE3f mVelocity;

  std::list<MapPoint*> mlpTemporalPoints;

  std::shared_ptr<GeometricCamera> mpCamera;
  std::shared_ptr<GeometricCamera> mpCamera2;

  int initID, lastID;

  Sophus::SE3f mTlr;

  void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
  bool Stop();

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;
#endif
};
typedef std::shared_ptr<Tracking> Tracking_ptr;
typedef std::weak_ptr<Tracking> Tracking_wptr;

}  // namespace MORB_SLAM

