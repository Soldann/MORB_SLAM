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
#include <memory>
#include <thread>

#include "MORB_SLAM/Tracking.h"
#include "MORB_SLAM/ImprovedTypes.hpp"
#include "MORB_SLAM/FrameDrawer.h"
#include "MORB_SLAM/MapDrawer.h"
#include "MORB_SLAM/Settings.h"
#include "MORB_SLAM/Packet.hpp"

namespace MORB_SLAM {

class Viewer {
  void newParameterLoader(const Settings& settings);
  // Main thread function. Draw points, keyframes, the current camera pose and
  // the last processed frame. Drawing is refreshed according to the camera fps.
  // We use Pangolin.
  void Run();
 public:

  Viewer(const System_ptr &pSystem, const std::string &strSettingPath);
  Viewer(const System_ptr &pSystem, const Settings &settings);

  virtual ~Viewer();

  void update(const Packet &pose);

  void close();
  bool isClosed() const;
  bool isOpen() const;


 private:
  void setBoth(const bool b);
  bool ParseViewerParamFile(cv::FileStorage& fSettings);

  bool Stop();

  System_ptr mpSystem;
  FrameDrawer mpFrameDrawer;
  MapDrawer mpMapDrawer;
  Tracking_ptr mpTracker;
  std::thread mptViewer;
  bool both;

  // 1/fps in ms
  double mT;
  float mImageWidth, mImageHeight;
  float mImageViewerScale;

  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

  bool mbClosed;
};

}  // namespace MORB_SLAM
