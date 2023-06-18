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

#include "MORB_SLAM/Atlas.h"

#include <iostream>
#include <mutex>
#include "MORB_SLAM/CameraModels/GeometricCamera.h"
#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"

namespace MORB_SLAM {

Atlas::Atlas() { mpCurrentMap = nullptr; }

Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid) {
  mpCurrentMap = nullptr;
  CreateNewMap();
}

Atlas::~Atlas() {}

void Atlas::CreateNewMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  std::cout << "Creation of new std::map with id: " << Map::nNextId << std::endl;
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1;  // The init KF is the next of current maximum

    mpCurrentMap->SetStoredMap();
    std::cout << "Stored std::map with ID: " << mpCurrentMap->GetId() << std::endl;
  }
  std::cout << "Creation of new std::map with last KF id: " << mnLastInitKFidMap << std::endl;

  mpCurrentMap = std::make_shared<Map>(mnLastInitKFidMap);
  mpCurrentMap->SetCurrentMap();
  mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(std::shared_ptr<Map> pMap) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  std::cout << "Change to std::map with id: " << pMap->GetId() << std::endl;
  if (mpCurrentMap) {
    mpCurrentMap->SetStoredMap();
  }

  mpCurrentMap = pMap;
  mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mnLastInitKFidMap;
}

void Atlas::AddKeyFrame(KeyFrame* pKF) {
  std::shared_ptr<Map> pMapKF = pKF->GetMap();
  pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP) {
  std::shared_ptr<Map> pMapMP = pMP->GetMap();
  pMapMP->AddMapPoint(pMP);
}

std::shared_ptr<GeometricCamera> Atlas::AddCamera(const std::shared_ptr<GeometricCamera> &pCam) {
  // Check if the camera already exists
  bool bAlreadyInMap = false;
  int index_cam = -1;
  for (size_t i = 0; i < mvpCameras.size(); ++i) {
    std::shared_ptr<GeometricCamera> pCam_i = mvpCameras[i];
    if (!pCam) std::cout << "Not pCam" << std::endl;
    if (!pCam_i) std::cout << "Not pCam_i" << std::endl;
    if (pCam->GetType() != pCam_i->GetType()) continue;

    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      if (std::reinterpret_pointer_cast<Pinhole>(pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
      }
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      if (std::reinterpret_pointer_cast<KannalaBrandt8>(pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
      }
    }
  }

  if (bAlreadyInMap) {
    return mvpCameras[index_cam];
  } else {
    mvpCameras.push_back(pCam);
    return pCam;
  }
}

std::vector<std::shared_ptr<GeometricCamera>> Atlas::GetAllCameras() { return mvpCameras; }

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetReferenceMapPoints();
}

std::vector<std::shared_ptr<Map>> Atlas::GetAllMaps() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  struct compFunctor {
    inline bool operator()(std::shared_ptr<Map> elem1, std::shared_ptr<Map> elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::vector<std::shared_ptr<Map>> vMaps(mspMaps.begin(), mspMaps.end());
  sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

int Atlas::CountMaps() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

void Atlas::clearMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end();
  it!=send; it++)
  {
      (*it)->clear();
      delete *it;
  }*/
  mspMaps.clear();
  mpCurrentMap = nullptr;
  mnLastInitKFidMap = 0;
}

std::shared_ptr<Map> Atlas::GetCurrentMap(System* sys) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  if (!mpCurrentMap) CreateNewMap();
  while (mpCurrentMap->IsBad()) {
    if (sys != nullptr) return nullptr;
  }

  return mpCurrentMap;
}

void Atlas::SetMapBad(std::shared_ptr<Map> pMap) {
  // mspMaps.erase(pMap);
  pMap->SetBad();

  mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps() {
  /*for(Map* pMap : mspBadMaps)
  {
      delete pMap;
      pMap = nullptr;
  }*/
  mspBadMaps.clear();
}

bool Atlas::isInertial() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave() {
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1;  // The init KF is the next of current maximum
  }

  struct compFunctor {
    inline bool operator()(std::shared_ptr<Map> elem1, std::shared_ptr<Map> elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
  sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());
  std::set<std::shared_ptr<GeometricCamera>> spCams(mvpCameras.begin(), mvpCameras.end());

  for (std::shared_ptr<Map> pMi : mvpBackupMaps) {
    if (!pMi || pMi->IsBad()) continue;

    if (pMi->GetAllKeyFrames().size() == 0) {
      // Empty map, erase before of save it.
      SetMapBad(pMi);
      continue;
    }
    pMi->PreSave(spCams, pMi);
  }
  RemoveBadMaps();
}

void Atlas::PostLoad() {
  std::map<unsigned int, std::shared_ptr<GeometricCamera>> mpCams;
  for (std::shared_ptr<GeometricCamera> pCam : mvpCameras) {
    mpCams[pCam->GetId()] = pCam;
  }

  mspMaps.clear();
  unsigned long int numKF = 0, numMP = 0;
  for (std::shared_ptr<Map> pMi : mvpBackupMaps) {
    mspMaps.insert(pMi);
    pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams, pMi);
    numKF += pMi->GetAllKeyFrames().size();
    numMP += pMi->GetAllMapPoints().size();
  }
  mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB) {
  mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase() { return mpKeyFrameDB; }

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc) {
  mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary() { return mpORBVocabulary; }

long unsigned int Atlas::GetNumLivedKF() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (std::shared_ptr<Map> pMap_i : mspMaps) {
    num += pMap_i->GetAllKeyFrames().size();
  }

  return num;
}

long unsigned int Atlas::GetNumLivedMP() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (std::shared_ptr<Map> pMap_i : mspMaps) {
    num += pMap_i->GetAllMapPoints().size();
  }

  return num;
}

std::map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes() {
  std::map<long unsigned int, KeyFrame*> mpIdKFs;
  for (std::shared_ptr<Map>  pMap_i : mvpBackupMaps) {
    std::vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

    for (KeyFrame* pKF_j_Mi : vpKFs_Mi) {
      mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
    }
  }

  return mpIdKFs;
}

}  // namespace MORB_SLAM
