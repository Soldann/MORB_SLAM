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

#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <mutex>
#include <set>
#include <map>
#include <vector>

#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/KeyFrame.h"
#include "MORB_SLAM/Map.h"
#include "MORB_SLAM/MapPoint.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"

namespace MORB_SLAM {
class Map;
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;

// BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
// BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

class Atlas {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar.template register_type<Pinhole>();
    ar.template register_type<KannalaBrandt8>();

    // Save/load a set structure, the set structure is broken in libboost 1.58
    // for ubuntu 16.04, a vector is serializated
    // ar & mspMaps;
    ar& mvpBackupMaps;
    ar& mvpCameras;
    // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
    ar& Map::nNextId;
    ar& Frame::nNextId;
    ar& KeyFrame::nNextId;
    ar& MapPoint::nNextId;
    ar& GeometricCamera::nNextId;
    ar& mnLastInitKFidMap;
  }

  std::deque<Sophus::SE3f> pValues;
  int qSize;

 public:
  

  Atlas();
  Atlas(int initKFid);  // When its initialization the first map is created
  ~Atlas();

  void CreateNewMap();
  void ChangeMap(std::shared_ptr<Map> pMap);

  unsigned long int GetLastInitKFid();

  // Method for change components in the current map
  void AddKeyFrame(KeyFrame* pKF);
  void AddMapPoint(MapPoint* pMP);
  // void EraseMapPoint(MapPoint* pMP);
  // void EraseKeyFrame(KeyFrame* pKF);

  std::shared_ptr<GeometricCamera> AddCamera(const std::shared_ptr<GeometricCamera> &pCam);
  std::vector<std::shared_ptr<GeometricCamera>> GetAllCameras();

  /* All methods without Map pointer work on current map */
  void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);
  void InformNewBigChange();
  int GetLastBigChangeIdx();

  long unsigned int MapPointsInMap();
  long unsigned KeyFramesInMap();

  // Method for get data in current map
  std::vector<KeyFrame*> GetAllKeyFrames();
  std::vector<MapPoint*> GetAllMapPoints();
  std::vector<MapPoint*> GetReferenceMapPoints();

  std::vector<std::shared_ptr<Map>> GetAllMaps();

  int CountMaps();

  void clearMap();

  void clearAtlas();

  std::shared_ptr<Map> GetCurrentMap(System* sys = nullptr);

  void SetMapBad(std::shared_ptr<Map> pMap);
  void RemoveBadMaps();

  bool isInertial();
  void SetInertialSensor();
  void SetImuInitialized();
  bool isImuInitialized();

  // Function for garantee the correction of serialization of this object
  void PreSave();
  void PostLoad();

  std::map<long unsigned int, KeyFrame*> GetAtlasKeyframes();

  void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
  KeyFrameDatabase* GetKeyFrameDatabase();

  void SetORBVocabulary(ORBVocabulary* pORBVoc);
  ORBVocabulary* GetORBVocabulary();

  long unsigned int GetNumLivedKF();

  long unsigned int GetNumLivedMP();

  std::deque<Sophus::SE3f> &getPoseQueue();
  int getQueueSize();
  int getMaxChange();

  void addPoseToQueue(Sophus::SE3f poseCandidate);

  void setPoseOffset(Sophus::SE3f pose);
  Sophus::SE3f getPoseOffset();


 

 protected:
  std::set<std::shared_ptr<Map>> mspMaps;
  std::set<std::shared_ptr<Map>> mspBadMaps;
  // Its necessary change the container from set to vector because libboost 1.58
  // and Ubuntu 16.04 have an error with this cointainer
  std::vector<std::shared_ptr<Map>> mvpBackupMaps;

  std::shared_ptr<Map> mpCurrentMap;

  std::vector<std::shared_ptr<GeometricCamera>> mvpCameras;

  unsigned long int mnLastInitKFidMap;

  // Class references for the map reconstruction from the save file
  KeyFrameDatabase* mpKeyFrameDB;
  ORBVocabulary* mpORBVocabulary;

  // Mutex
  std::mutex mMutexAtlas;

  std::deque<Sophus::SE3f> poseValues;
  const float maxChangeInPose = 1.5;
  const int queueSize = 5;

  Sophus::SE3f poseOffset;

};  // class Atlas

}  // namespace MORB_SLAM
