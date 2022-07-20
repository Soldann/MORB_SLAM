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

#include "Viewer.h"

#include <pangolin/pangolin.h>

#include <chrono>
#include <ctime>
#include <mutex>
#include <stdexcept>
#include <iostream>

namespace ORB_SLAM3 {

void Viewer::setBoth(const bool b){
  both = true;
  mpFrameDrawer.both = true;
}

Viewer::Viewer(const System_ptr &pSystem, const std::string &strSettingPath)
    : both(false),
      mpSystem(pSystem),
      mpFrameDrawer(pSystem->mpAtlas),
      mpMapDrawer(pSystem->mpAtlas, strSettingPath),
      mpTracker(pSystem->mpTracker),
      mbClosed(false),
      mbStopTrack(false) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if (!ParseViewerParamFile(fSettings)) {
      std::cerr << "**ERROR in the config file, the format is not correct**"
                << std::endl;
      throw std::runtime_error("**ERROR in the config file, the format is not correct**");
    }
    mptViewer = std::thread(&Viewer::Run, this);
    both = mpFrameDrawer.both;
}

Viewer::Viewer(const System_ptr &pSystem, const Settings &settings)
    : both(false),
      mpSystem(pSystem),
      mpFrameDrawer(pSystem->mpAtlas),
      mpMapDrawer(pSystem->mpAtlas, settings),
      mpTracker(pSystem->mpTracker),
      mbClosed(true),
      mbStopTrack(false) {
    newParameterLoader(settings);
    mptViewer = std::thread(&Viewer::Run, this);
}

Viewer::~Viewer(){
  close();
  if(mptViewer.joinable()) mptViewer.join();
}

void Viewer::newParameterLoader(const Settings &settings) {
  mImageViewerScale = 1.f;

  float fps = settings.fps();
  if (fps < 1) fps = 30;
  mT = 1e3 / fps;

  cv::Size imSize = settings.newImSize();
  mImageHeight = imSize.height;
  mImageWidth = imSize.width;

  mImageViewerScale = settings.imageViewerScale();
  mViewpointX = settings.viewPointX();
  mViewpointY = settings.viewPointY();
  mViewpointZ = settings.viewPointZ();
  mViewpointF = settings.viewPointF();

  if ((mpTracker->mSensor == System::STEREO || mpTracker->mSensor == System::IMU_STEREO ||
        mpTracker->mSensor == System::IMU_RGBD || mpTracker->mSensor == System::RGBD) &&
        settings.cameraType() == Settings::KannalaBrandt)
        setBoth(true);
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings) {
  bool b_miss_params = false;
  mImageViewerScale = 1.f;

  float fps = fSettings["Camera.fps"];
  if (fps < 1) fps = 30;
  mT = 1e3 / fps;

  cv::FileNode node = fSettings["Camera.width"];
  if (!node.empty()) {
    mImageWidth = node.real();
  } else {
    std::cerr
        << "*Camera.width parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.height"];
  if (!node.empty()) {
    mImageHeight = node.real();
  } else {
    std::cerr
        << "*Camera.height parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.imageViewScale"];
  if (!node.empty()) {
    mImageViewerScale = node.real();
  }

  node = fSettings["Viewer.ViewpointX"];
  if (!node.empty()) {
    mViewpointX = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.ViewpointY"];
  if (!node.empty()) {
    mViewpointY = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.ViewpointZ"];
  if (!node.empty()) {
    mViewpointZ = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.ViewpointF"];
  if (!node.empty()) {
    mViewpointF = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  if(!b_miss_params){
    string sCameraName = fSettings["Camera.type"];
    if ((mpTracker->mSensor == System::STEREO || mpTracker->mSensor == System::IMU_STEREO ||
          mpTracker->mSensor == System::IMU_RGBD || mpTracker->mSensor == System::RGBD) &&
          sCameraName == "KannalaBrandt8")
          setBoth(true);
  }

  return !b_miss_params;
}

void Viewer::update(const Sophus::SE3f &pose){
  if(mpTracker->mState != NOT_INITIALIZED){
    mpFrameDrawer->Update(mpTracker);
    mpMapDrawer->SetCurrentCameraPose(pose);
  }
}

void Viewer::Run() {

  pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer", 1024, 768);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
  pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
  pangolin::Var<bool> menuTopView("menu.Top View", false, false);
  // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
  pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
  pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph", true,
                                            true);
  pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false,
                                           true);
  pangolin::Var<bool> menuReset("menu.Reset", false, false);
  pangolin::Var<bool> menuStop("menu.Stop", false, false);

  pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0,
                                0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc, Twr;
  Twc.SetIdentity();
  pangolin::OpenGlMatrix Ow;  // Oriented with g in the z axis
  Ow.SetIdentity();
  cv::namedWindow("ORB-SLAM3: Current Frame");

  bool bFollow = true;
  bool bLocalizationMode = false;
  bool bCameraView = true;

  if (mpTracker->mSensor == mpSystem->MONOCULAR ||
      mpTracker->mSensor == mpSystem->STEREO ||
      mpTracker->mSensor == mpSystem->RGBD) {
    menuShowGraph = true;
  }

  float trackedImageScale = mpTracker->GetImageScale();

  cout << "Starting the Viewer" << endl;
  while (isOpen()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);

    if (menuFollowCamera && bFollow) {
      if (bCameraView)
        s_cam.Follow(Twc);
      else
        s_cam.Follow(Ow);
    } else if (menuFollowCamera && !bFollow) {
      if (bCameraView) {
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
            mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
        s_cam.Follow(Twc);
      } else {
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(
            pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
        s_cam.Follow(Ow);
      }
      bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    if (menuCamView) {
      menuCamView = false;
      bCameraView = true;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
          1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
          mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc);
    }

    if (menuTopView && mpMapDrawer->mpAtlas->isImuInitialized()) {
      menuTopView = false;
      bCameraView = false;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
          1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(
          pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
      s_cam.Follow(Ow);
    }

    if (menuLocalizationMode && !bLocalizationMode) {
      mpSystem->ActivateLocalizationMode();
      bLocalizationMode = true;
    } else if (!menuLocalizationMode && bLocalizationMode) {
      mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    mpMapDrawer->DrawCurrentCamera(Twc);
    if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph ||
        menuShowOptLba)
      mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph,
                                 menuShowInertialGraph, menuShowOptLba);
    if (menuShowPoints) mpMapDrawer->DrawMapPoints();

    pangolin::FinishFrame();

    cv::Mat toShow;
    cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

    if (both) {
      cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
      cv::hconcat(im, imRight, toShow);
    } else {
      toShow = im;
    }

    if (mImageViewerScale != 1.f) {
      int width = toShow.cols * mImageViewerScale;
      int height = toShow.rows * mImageViewerScale;
      cv::resize(toShow, toShow, cv::Size(width, height));
    }

    cv::imshow("ORB-SLAM3: Current Frame", toShow);
    cv::waitKey(mT);

    if (menuReset) {
      menuShowGraph = true;
      menuShowInertialGraph = true;
      menuShowKeyFrames = true;
      menuShowPoints = true;
      menuLocalizationMode = false;
      if (bLocalizationMode) mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
      bFollow = true;
      menuFollowCamera = true;
      mpSystem->ResetActiveMap();
      menuReset = false;
    }

    if (menuStop)
      break;
  }

  close();
}

bool Viewer::isClosed() const {
  return mbClosed;
}

bool Viewer::isOpen() const {
  return !mbClosed;
}

void Viewer::close(){
  mbClosed = true;
}

}  // namespace ORB_SLAM3
