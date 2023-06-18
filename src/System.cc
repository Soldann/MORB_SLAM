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

#include "MORB_SLAM/System.h"

#include <openssl/md5.h>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "MORB_SLAM/ImprovedTypes.hpp"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>
#include <string>
#include <iostream>

#include "MORB_SLAM/Converter.h"

namespace MORB_SLAM {

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const std::string& strVocFile, const std::string& strSettingsFile,
               const CameraType::eSensor sensor,
               const std::string& strSequence)
    : mSensor(sensor),
      mpAtlas(std::make_shared<Atlas>(0)),
      mbReset(false),
      mbResetActiveMap(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false) {

  cameras.push_back(std::make_shared<Camera>(mSensor)); // for now just hard code the sensor we are using, TODO make multicam
  // Output welcome message
  std::cout << std::endl
       << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, "
          "Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of "
          "Zaragoza."
       << std::endl
       << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel "
          "and Juan D. Tardós, University of Zaragoza."
       << std::endl
       << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl
       << "This is free software, and you are welcome to redistribute it"
       << std::endl
       << "under certain conditions. See LICENSE.txt." << std::endl
       << std::endl;

  std::cout << "Input sensor was set to: ";

  if (mSensor == CameraType::MONOCULAR)
    std::cout << "Monocular" << std::endl;
  else if (mSensor == CameraType::STEREO)
    std::cout << "Stereo" << std::endl;
  else if (mSensor == CameraType::RGBD)
    std::cout << "RGB-D" << std::endl;
  else if (mSensor == CameraType::IMU_MONOCULAR)
    std::cout << "Monocular-Inertial" << std::endl;
  else if (mSensor == CameraType::IMU_STEREO)
    std::cout << "Stereo-Inertial" << std::endl;
  else if (mSensor == CameraType::IMU_RGBD)
    std::cout << "RGB-D-Inertial" << std::endl;

  // Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
    exit(-1);
  }

  cv::FileNode node = fsSettings["File.version"];
  if (!node.empty() && node.isString() && node.string() == "1.0") {
    settings_ = new Settings(strSettingsFile, mSensor);

    mStrLoadAtlasFromFile = settings_->atlasLoadFile();
    mStrSaveAtlasToFile = settings_->atlasSaveFile();

    // std::cout << (*settings_) << std::endl;
  } else {
    settings_ = nullptr;
    cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
    if (!node.empty() && node.isString()) {
      mStrLoadAtlasFromFile = (std::string)node;
    }

    node = fsSettings["System.SaveAtlasToFile"];
    if (!node.empty() && node.isString()) {
      mStrSaveAtlasToFile = (std::string)node;
    }
  }

  node = fsSettings["loopClosing"];
  bool activeLC = true;
  if (!node.empty()) {
    activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
  }

  mStrVocabularyFilePath = strVocFile;

  // bool loadedAtlas = false; // UNUSED

  bool isRead = false;
  if (mStrLoadAtlasFromFile.empty()) {
    // Load ORB Vocabulary
    std::cout << std::endl
         << "Loading ORB Vocabulary. This could take a while..." << std::endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
      std::cerr << "Wrong path to vocabulary. " << std::endl;
      std::cerr << "Falied to open at: " << strVocFile << std::endl;
      exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Atlas
    std::cout << "Initialization of Atlas from scratch " << std::endl;
    // mpAtlas = new Atlas(0);
  } else {
    // Load ORB Vocabulary
    std::cout << std::endl
         << "Loading ORB Vocabulary. This could take a while..." << std::endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
      std::cerr << "Wrong path to vocabulary. " << std::endl;
      std::cerr << "Falied to open at: " << strVocFile << std::endl;
      exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    std::cout << "Load File" << std::endl;

    // Load the file with an earlier session
    // clock_t start = clock();

    std::cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile
         << std::endl;
    isRead = LoadAtlas(FileType::BINARY_FILE);

    if (!isRead) {
      std::cout << "Error to load the file, please try with other session file or "
              "vocabulary file"
           << std::endl;
      exit(-1);
    }
    // mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // std::cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " <<
    // mpKeyFrameDatabase->mnNumWords << std::endl;

    // loadedAtlas = true; // UNUSED

    mpAtlas->CreateNewMap();

    // clock_t timeElapsed = clock() - start;
    // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
    // std::cout << "Binary file read in " << msElapsed << " ms" << std::endl;

    // usleep(10*1000*1000);
  }

  if (mSensor == CameraType::IMU_STEREO || mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_RGBD)
    mpAtlas->SetInertialSensor();

  // Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this
  // constructor)
  std::cout << "Seq. Name: " << strSequence << std::endl;
  mpTracker = new Tracking(this, mpVocabulary,
                           mpAtlas, mpKeyFrameDatabase, strSettingsFile,
                           mSensor, settings_, strSequence);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(
      this, mpAtlas, mSensor == CameraType::MONOCULAR || mSensor == CameraType::IMU_MONOCULAR,
      mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO || mSensor == CameraType::IMU_RGBD,
      strSequence);
  
  // Do not axis flip when loading from existing atlas
  if (isRead) {
    mpLocalMapper->setIsDoneVIBA(true);
    mpLocalMapper->setNotifyIsDoneVIBA(true);
  }

  mptLocalMapping = new std::thread(&MORB_SLAM::LocalMapping::Run, mpLocalMapper);
  if (settings_)
    mpLocalMapper->mThFarPoints = settings_->thFarPoints();
  else
    mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
  if (mpLocalMapper->mThFarPoints != 0) {
    std::cout << "Discard points further than " << mpLocalMapper->mThFarPoints
         << " m from current camera" << std::endl;
    mpLocalMapper->mbFarPoints = true;
  } else
    mpLocalMapper->mbFarPoints = false;

  // Initialize the Loop Closing thread and launch
  // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
  mpLoopCloser =
      new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary,
                      mSensor != CameraType::MONOCULAR, activeLC);  // mSensor!=CameraType::MONOCULAR);
  mptLoopClosing = new std::thread(&MORB_SLAM::LoopClosing::Run, mpLoopCloser);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // Fix verbosity
  Verbose::SetTh(Verbose::VERBOSITY_QUIET);
}

Sophus::SE3f System::TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight,
                                 const double& timestamp,
                                 const std::vector<IMU::Point>& vImuMeas,
                                 std::string filename) {
  if (mSensor != CameraType::STEREO && mSensor != CameraType::IMU_STEREO) {
    std::cerr << "ERROR: you called TrackStereo but input sensor was not set to "
            "Stereo nor Stereo-Inertial."
         << std::endl;
    exit(-1);
  }

  cv::Mat imLeftToFeed, imRightToFeed;
  if (settings_ && settings_->needToRectify()) {
    const cv::Mat &M1l = settings_->M1l();
    const cv::Mat &M2l = settings_->M2l();
    const cv::Mat &M1r = settings_->M1r();
    const cv::Mat &M2r = settings_->M2r();

    cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
  } else if (settings_ && settings_->needToResize()) {
    cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
    cv::resize(imRight, imRightToFeed, settings_->newImSize());
  } else {
    imLeftToFeed = imLeft;
    imRightToFeed = imRight;
  }

  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
      mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
    }
  }

  if (mSensor == CameraType::IMU_STEREO)
    for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
      mpTracker->GrabImuData(vImuMeas[i_imu]);

  // std::cout << "start GrabImageStereo" << std::endl;
  Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed,
                                                timestamp, filename, cameras[0]); // for now we know cameras[0] is providing the image

  // Eigen::Vector3f vel = mpTracker->mCurrentFrame.GetVelocity();

  // std::cout << "out grabber" << std::endl;

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

  return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat& im, const cv::Mat& depthmap,
                               const double& timestamp,
                               const std::vector<IMU::Point>& vImuMeas,
                               std::string filename) {
  if (mSensor != CameraType::RGBD && mSensor != CameraType::IMU_RGBD) {
    std::cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD."
         << std::endl;
    exit(-1);
  }

  cv::Mat imToFeed = im.clone();
  cv::Mat imDepthToFeed = depthmap.clone();
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;

    cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
  }

  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
      mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
    }
  }

  if (mSensor == CameraType::IMU_RGBD)
    for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
      mpTracker->GrabImuData(vImuMeas[i_imu]);

  Sophus::SE3f Tcw =
      mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename, cameras[0]); // for now we know cameras[0] is providing the image

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat& im, const double& timestamp,
                                    const std::vector<IMU::Point>& vImuMeas,
                                    std::string filename) {
  // {
  //   std::unique_lock<std::mutex> lock(mMutexReset);
  //   if (mbShutDown) return Sophus::SE3f();
  // }

  if (mSensor != CameraType::MONOCULAR && mSensor != CameraType::IMU_MONOCULAR) {
    std::cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
            "Monocular nor Monocular-Inertial."
         << std::endl;
    exit(-1);
  }

  cv::Mat imToFeed = im.clone();
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;
  }

  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
      mbResetActiveMap = false;
    } else if (mbResetActiveMap) {
      std::cout << "SYSTEM-> Reseting active map in monocular case" << std::endl;
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
    }
  }

  if (mSensor == CameraType::IMU_MONOCULAR)
    for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
      mpTracker->GrabImuData(vImuMeas[i_imu]);

  Sophus::SE3f Tcw =
      mpTracker->GrabImageMonocular(imToFeed, timestamp, filename, cameras[0]); // for now we know cameras[0] is providing the image

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

  return Tcw;
}

void System::ActivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
  static int n = 0;
  int curn = mpAtlas->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else
    return false;
}

void System::Reset() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbReset = true;
}

void System::ResetActiveMap() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

System::~System() {
  std::cout << "Shutdown" << std::endl;

  std::unique_lock<std::mutex> lock(mMutexReset);

  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();

  // Wait until all thread have effectively stopped
  /*while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
         mpLoopCloser->isRunningGBA()) {
    if (!mpLocalMapper->isFinished())
      std::cout << "mpLocalMapper is not finished" << std::endl;
    if (!mpLoopCloser->isFinished())
      std::cout << "mpLoopCloser is not finished" << std::endl;
    if (mpLoopCloser->isRunningGBA()) {
      std::cout << "mpLoopCloser is running GBA" << std::endl;
      std::cout << "break anyway..." << std::endl;
      // break;
    }
    usleep(5000);
  }*/

  if (!mStrSaveAtlasToFile.empty()) {
    Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile,
                       Verbose::VERBOSITY_DEBUG);
    SaveAtlas(FileType::BINARY_FILE);
  }
  // if (mptLocalMapping->joinable()) {
  //   mptLocalMapping->join();
  // }
  // if (mptLoopClosing->joinable()) {
  //   mptLoopClosing->join();
  // }

#ifdef REGISTER_TIMES
  mpTracker->PrintTimeStats();
#endif
}

void System::SaveTrajectoryTUM(const std::string& filename) {
  std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
  if (mSensor == CameraType::MONOCULAR) {
    std::cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << std::endl;
    return;
  }

  std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  std::list<MORB_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for (std::list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;

    KeyFrame* pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    f << std::setprecision(6) << *lT << " " << std::setprecision(9) << twc(0) << " "
      << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z()
      << " " << q.w() << std::endl;
  }
  f.close();
  // std::cout << std::endl << "trajectory saved!" << std::endl;
}

void System::SaveKeyFrameTrajectoryTUM(const std::string& filename) {
  std::cout << std::endl
       << "Saving keyframe trajectory to " << filename << " ... TUM" << std::endl;

  std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad()) continue;

    Sophus::SE3f Twc = pKF->GetPoseInverse();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    Eigen::Vector3f t = Twc.translation();
    f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t(0)
      << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << std::endl;
  }

  f.close();
}

void System::SaveTrajectoryEuRoC(const std::string& filename) {
  std::cout << std::endl << "Saving trajectory to " << filename << " ..." << std::endl;
  /*if(mSensor==CameraType::MONOCULAR)
  {
      std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  std::endl; return;
  }*/

  std::vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
  size_t numMaxKFs = 0;
  std::shared_ptr<Map> pBiggerMap = nullptr;
  std::cout << "There are " << std::to_string(vpMaps.size())
            << " maps in the atlas" << std::endl;
  for (std::shared_ptr<Map> pMap : vpMaps) {
    std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
              << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs"
              << std::endl;
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  if (pBiggerMap == nullptr) {
    std::cerr << "SaveTrajectoryEuRoC has no map to save a trajectory for!";
    return;
  }

  std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b depending on IMU or not.
  if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO || mSensor == CameraType::IMU_RGBD)
    Twb = vpKFs[0]->GetImuPose();
  else
    Twb = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  // std::cout << "file open" << std::endl;
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  std::list<MORB_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  // std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << std::endl;
  // std::cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << std::endl; std::cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << std::endl; std::cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << std::endl;

  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    // std::cout << "1" << std::endl;
    if (*lbL) continue;

    KeyFrame* pKF = *lRit;
    // std::cout << "KF: " << pKF->mnId << std::endl;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;

    // std::cout << "2.5" << std::endl;

    while (pKF->isBad()) {
      // std::cout << " 2.bad" << std::endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // std::cout << "--Parent KF: " << pKF->mnId << std::endl;
    }

    if (!pKF || pKF->GetMap() != pBiggerMap) {
      // std::cout << "--Parent KF is from another map" << std::endl;
      continue;
    }

    // std::cout << "3" << std::endl;

    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // std::cout << "4" << std::endl;

    if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO ||
        mSensor == CameraType::IMU_RGBD) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << std::endl;
    } else {
      Sophus::SE3f Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twc(0)
        << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << std::endl;
    }

    // std::cout << "5" << std::endl;
  }
  // std::cout << "end saving trajectory" << std::endl;
  f.close();
  std::cout << std::endl << "End of saving trajectory to " << filename << " ..." << std::endl;
}

void System::SaveTrajectoryEuRoC(const std::string& filename, std::shared_ptr<Map> pMap) {
  std::cout << std::endl
       << "Saving trajectory of map " << pMap->GetId() << " to " << filename
       << " ..." << std::endl;
  /*if(mSensor==CameraType::MONOCULAR)
  {
      std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  std::endl; return;
  }*/

  // int numMaxKFs = 0;

  std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b dependingo on IMU or not.
  if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO || mSensor == CameraType::IMU_RGBD)
    Twb = vpKFs[0]->GetImuPose();
  else
    Twb = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  // std::cout << "file open" << std::endl;
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  std::list<MORB_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  // std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << std::endl;
  // std::cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << std::endl; std::cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << std::endl; std::cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << std::endl;

  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    // std::cout << "1" << std::endl;
    if (*lbL) continue;

    KeyFrame* pKF = *lRit;
    // std::cout << "KF: " << pKF->mnId << std::endl;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;

    // std::cout << "2.5" << std::endl;

    while (pKF->isBad()) {
      // std::cout << " 2.bad" << std::endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // std::cout << "--Parent KF: " << pKF->mnId << std::endl;
    }

    if (!pKF || pKF->GetMap() != pMap) {
      // std::cout << "--Parent KF is from another map" << std::endl;
      continue;
    }

    // std::cout << "3" << std::endl;

    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // std::cout << "4" << std::endl;

    if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO ||
        mSensor == CameraType::IMU_RGBD) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << std::endl;
    } else {
      Sophus::SE3f Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twc(0)
        << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << std::endl;
    }

    // std::cout << "5" << std::endl;
  }
  // std::cout << "end saving trajectory" << std::endl;
  f.close();
  std::cout << std::endl << "End of saving trajectory to " << filename << " ..." << std::endl;
}

/*void System::SaveTrajectoryEuRoC(const std::string &filename)
{

    std::cout << std::endl << "Saving trajectory to " << filename << " ..." << std::endl;
    if(mSensor==CameraType::MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
std::endl; return;
    }

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or
not. if (mSensor==CameraType::IMU_MONOCULAR || mSensor==CameraType::IMU_STEREO || mSensor==CameraType::IMU_RGBD) Twb
= vpKFs[0]->GetImuPose_(); else Twb = vpKFs[0]->GetPoseInverse_();

    std::ofstream f;
    f.open(filename.c_str());
    // std::cout << "file open" << std::endl;
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    std::list<MORB_SLAM::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); std::list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); std::list<bool>::iterator lbL =
mpTracker->mlbLost.begin();

    //std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << std::endl;
    //std::cout << "size mlRelativeFramePoses: " <<
mpTracker->mlRelativeFramePoses.size() << std::endl;
    //std::cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size()
<< std::endl;
    //std::cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << std::endl;


    for(std::list<Sophus::SE3f>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++,
lT++, lbL++)
    {
        //std::cout << "1" << std::endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //std::cout << "KF: " << pKF->mnId << std::endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
get a suitable keyframe. if (!pKF) continue;

        //std::cout << "2.5" << std::endl;

        while(pKF->isBad())
        {
            //std::cout << " 2.bad" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //std::cout << "--Parent KF: " << pKF->mnId << std::endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //std::cout << "--Parent KF is from another map" << std::endl;
            continue;
        }

        //std::cout << "3" << std::endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new
world reference

        // std::cout << "4" << std::endl;


        if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO ||
mSensor==CameraType::IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) <<
twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
<< q.z() << " " << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) <<
twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
<< q.z() << " " << q.w() << std::endl;
        }

        // std::cout << "5" << std::endl;
    }
    //std::cout << "end saving trajectory" << std::endl;
    f.close();
    std::cout << std::endl << "End of saving trajectory to " << filename << " ..." <<
std::endl;
}*/

/*void System::SaveKeyFrameTrajectoryEuRoC_old(const std::string &filename)
{
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." <<
std::endl;

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO ||
mSensor==CameraType::IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            std::vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<
std::setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " <<
twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
std::endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            std::vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp << " " <<
std::setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " <<
t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
std::endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename) {
  std::cout << std::endl
       << "Saving keyframe trajectory to " << filename << " ... Euroc" << std::endl;

  std::vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
  std::shared_ptr<Map> pBiggerMap = nullptr;
  size_t numMaxKFs = 0;
  for (std::shared_ptr<Map> pMap : vpMaps) {
    if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  if (pBiggerMap == nullptr) {
    std::cout << "There is not a map!!" << std::endl;
    return;
  }

  std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
  std::cout << "sorted keyframes\n";

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (!pKF || pKF->isBad()) continue;
    if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO ||
        mSensor == CameraType::IMU_RGBD) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << std::endl;

    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << std::endl;
    }
  }
  f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename, std::shared_ptr<Map> pMap) {
  std::cout << std::endl
       << "Saving keyframe trajectory of map " << pMap->GetId() << " to "
       << filename << " ..." << std::endl;

  std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    if (!pKF || pKF->isBad()) continue;
    if (mSensor == CameraType::IMU_MONOCULAR || mSensor == CameraType::IMU_STEREO ||
        mSensor == CameraType::IMU_RGBD) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << std::endl;

    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << std::endl;
    }
  }
  f.close();
}

/*void System::SaveTrajectoryKITTI(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." <<
std::endl; if(mSensor==CameraType::MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." <<
std::endl; return;
    }

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    std::list<MORB_SLAM::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); std::list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); for(std::list<cv::Mat>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        MORB_SLAM::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << std::setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)
<< " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " <<
Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " << Rwc.at<float>(2,0) << "
" << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  <<
twc.at<float>(2) << std::endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const std::string& filename) {
  std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
  if (mSensor == CameraType::MONOCULAR) {
    std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << std::endl;
    return;
  }

  std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  std::list<MORB_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  for (std::list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++) {
    MORB_SLAM::KeyFrame* pKF = *lRit;

    Sophus::SE3f Trw;

    if (!pKF) continue;

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Tow;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix3f Rwc = Twc.rotationMatrix();
    Eigen::Vector3f twc = Twc.translation();

    f << std::setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2)
      << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " "
      << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1)
      << " " << Rwc(2, 2) << " " << twc(2) << std::endl;
  }
  f.close();
}

void System::SaveDebugData(const int& initIdx) {
  // 0. Save initialization trajectory
  SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                      std::to_string(mpLocalMapper->mInitSect) + "_" +
                      std::to_string(initIdx) + ".txt");

  // 1. Save scale
  std::ofstream f;
  f.open("init_Scale_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
         std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mScale << std::endl;
  f.close();

  // 2. Save gravity direction
  f.open("init_GDir_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
         std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
    << mpLocalMapper->mRwg(0, 2) << std::endl;
  f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
    << mpLocalMapper->mRwg(1, 2) << std::endl;
  f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
    << mpLocalMapper->mRwg(2, 2) << std::endl;
  f.close();

  // 3. Save computational cost
  f.open("init_CompCost_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
         std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mCostTime << std::endl;
  f.close();

  // 4. Save biases
  f.open("init_Biases_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
         std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
    << mpLocalMapper->mbg(2) << std::endl;
  f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
    << mpLocalMapper->mba(2) << std::endl;
  f.close();

  // 5. Save covariance matrix
  f.open("init_CovMatrix_" + std::to_string(mpLocalMapper->mInitSect) + "_" +
             std::to_string(initIdx) + ".txt",
         std::ios_base::app);
  f << std::fixed;
  for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++) {
    for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++) {
      if (j != 0) f << ",";
      f << std::setprecision(15) << mpLocalMapper->mcovInertial(i, j);
    }
    f << std::endl;
  }
  f.close();

  // 6. Save initialization time
  f.open("init_Time_" + std::to_string(mpLocalMapper->mInitSect) + ".txt",
         std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mInitTime << std::endl;
  f.close();
}

Tracker::eTrackingState System::GetTrackingState() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackingState;
}

std::vector<MapPoint*> System::GetTrackedMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit() {
  double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  if ((aux > 0.) && mpAtlas->isImuInitialized())
    return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  else
    return 0.f;
}

bool System::isLost() {
  if (!mpAtlas->isImuInitialized())
    return false;
  else {
    if ((mpTracker->mState ==
         Tracker::LOST))  //||(mpTracker->mState==Tracker::RECENTLY_LOST))
      return true;
    else
      return false;
  }
}

bool System::isFinished() { return (GetTimeFromIMUInit() > 0.1); }

void System::ChangeDataset() {
  if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12) {
    mpTracker->ResetActiveMap();
  } else {
    mpTracker->CreateMapInAtlas();
  }

  mpTracker->NewDataset();
}

float System::GetImageScale() { return mpTracker->GetImageScale(); }

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time) {
  mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time) {
  mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time) {
  mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

void System::SaveAtlas(int type) {
  std::cout << "Thread ID is: " << std::this_thread::get_id() << std::endl;
  std::cout << "trying to save \n";
  if (!mStrSaveAtlasToFile.empty()) {
    std::cout << "not empty\n";
    // clock_t start = clock();

    // Save the current session
    mpAtlas->PreSave();
    std::cout << "presaved\n";
    std::string pathSaveFileName = mStrSaveAtlasToFile;  
    auto time = std::chrono::system_clock::now();
    std::time_t time_time = std::chrono::system_clock::to_time_t(time);
    std::string str_time = std::ctime(&time_time);
    pathSaveFileName = pathSaveFileName.append(".osa");

    std::cout << "About to Calculate \n";

    std::string strVocabularyChecksum =
        CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

    std::cout << "Vocab checksum`" << strVocabularyChecksum << std::endl;
    std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
    std::string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);
    std::cout << "Type is of: " << type << std::endl;
    if (type == TEXT_FILE)  // File text
    {
      std::cout << "Starting to write the save text file " << std::endl;

      int rval = std::remove(pathSaveFileName.c_str());  // Deletes the file
      std::cout << "remove's output is: " << rval << std::endl;

      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << *mpAtlas;
      std::cout << "End to write the save text file" << std::endl;
    } else if (type == BINARY_FILE)  // File binary
    {
      std::cout << "Starting to write the save binary file" << std::endl;
      int rval = std::remove(pathSaveFileName.c_str());  // Deletes the file
      std::cerr << errno << std::endl;
      std::cout << "remove's output is: " << rval << std::endl;
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      std::cout << "big boostin' time\n";
      boost::archive::binary_oarchive oa(ofs);
      std::cout << "streaming\n";
      oa << strVocabularyName;
      std::cout << "streamed name\n";
      oa << strVocabularyChecksum;
      std::cout << "streamed checksum\n";
      oa << *mpAtlas;
      std::cout << "End to write save binary file" << std::endl;
    } else {
      std::cout << "no file to be saved I guess lul\n";
    }
  }
}

bool System::LoadAtlas(int type) {
  std::string strFileVoc, strVocChecksum;
  bool isRead = false;

  std::string pathLoadFileName = "/";
  pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
  pathLoadFileName = pathLoadFileName.append(".osa");

  if (type == TEXT_FILE)  // File text
  {
    std::cout << "Starting to read the save text file " << std::endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      std::cout << "Load file not found" << std::endl;
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> *mpAtlas;
    std::cout << "End to load the save text file " << std::endl;
    isRead = true;
  } else if (type == BINARY_FILE)  // File binary
  {
    std::cout << "Starting to read the save binary file" << std::endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      std::cout << "Load file not found" << std::endl;
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> *mpAtlas;
    std::cout << "End to load the save binary file" << std::endl;
    isRead = true;
  }

  if (isRead) {
    // Check if the vocabulary is the same
    std::string strInputVocabularyChecksum =
        CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      std::cout << "The vocabulary load isn't the same which the load session was "
              "created "
           << std::endl;
      std::cout << "-Vocabulary name: " << strFileVoc << std::endl;
      return false;  // Both are differents
    }

    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();
    return true;
  }
  return false;
}

std::string System::CalculateCheckSum(std::string filename, int type) {
  std::string checksum = "";

  unsigned char c[MD5_DIGEST_LENGTH];

  std::ios_base::openmode flags = std::ios::in;
  if (type == BINARY_FILE)  // Binary file
    flags = std::ios::in | std::ios::binary;

  std::cout << "inside\n";

  std::ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    std::cout << "[E] Unable to open the in file " << filename << " for Md5 hash."
         << std::endl;
    return checksum;
  }

  MD5_CTX md5Context;
  char buffer[1024];

  std::cout << "buffer generated\n";

  MD5_Init(&md5Context);

  std::cout << "just initialized MD5\n";
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    MD5_Update(&md5Context, buffer, count);
    // std::cout << buffer;
  }
  std::cout << "about to close\n";

  f.close();

  MD5_Final(c, &md5Context);

  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    char aux[10];
    sprintf(aux, "%02x", c[i]);
    checksum = checksum + aux;
  }

  return checksum;
}

void System::setTrackingState(Tracker::eTrackingState state) {
  mpTracker->mState = state;
}

bool System::getHasMergedLocalMap() { 
  return mpLoopCloser->hasMergedLocalMap; 
}

bool System::getIsDoneVIBA() {
  return mpLocalMapper->getIsDoneVIBA();
}

}  // namespace MORB_SLAM
