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

#include "MORB_SLAM/Settings.h"

#include "MORB_SLAM/ImprovedTypes.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

#include "MORB_SLAM/CameraModels/KannalaBrandt8.h"
#include "MORB_SLAM/CameraModels/Pinhole.h"
#include <stdexcept>

namespace MORB_SLAM {

template <>
float Settings::readParameter<float>(cv::FileStorage& fSettings,
                                     const std::string& name, bool& found,
                                     const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      std::cerr << name << " optional parameter does not exist..." << std::endl;
      found = false;
      return 0.0f;
    }
  } else if (!node.isReal()) {
    std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
    throw std::invalid_argument(name + " parameter must be a real number, aborting...");
  } else {
    found = true;
    return node.real();
  }
}

template <>
int Settings::readParameter<int>(cv::FileStorage& fSettings,
                                 const std::string& name, bool& found,
                                 const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      std::cerr << name << " optional parameter does not exist..." << std::endl;
      found = false;
      return 0;
    }
  } else if (!node.isInt()) {
    std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
    throw std::invalid_argument(name + " parameter must be an integer number, aborting...");
  } else {
    found = true;
    return node.operator int();
  }
}

template <>
std::string Settings::readParameter<std::string>(cv::FileStorage& fSettings,
                                       const std::string& name, bool& found,
                                       const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      std::cerr << name << " optional parameter does not exist..." << std::endl;
      found = false;
      return std::string();
    }
  } else if (!node.isString()) {
    std::cerr << name << " parameter must be a std::string, aborting..." << std::endl;
    throw std::invalid_argument(name + " parameter must be an integer number, aborting...");
  } else {
    found = true;
    return node.string();
  }
}

template <>
cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage& fSettings,
                                         const std::string& name, bool& found,
                                         const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
      throw std::invalid_argument(name + " required parameter does not exist, aborting...");
    } else {
      std::cerr << name << " optional parameter does not exist..." << std::endl;
      found = false;
      return cv::Mat();
    }
  } else {
    found = true;
    return node.mat();
  }
}

Settings::Settings(const std::string& configFile, const CameraType& sensor)
    : sensor_(sensor),
      bNeedToUndistort_(false),
      bNeedToRectify_(false),
      bNeedToResize1_(false),
      bNeedToResize2_(false) {

  // Open settings file
  cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    std::cerr << "[ERROR]: could not open configuration file at: " << configFile
         << std::endl;
    std::cerr << "Aborting..." << std::endl;

    throw std::invalid_argument("[ERROR]: could not open configuration file at: " + configFile);
  } else {
    std::cout << "Loading settings from " << configFile << std::endl;
  }

  // Read first camera
  readCamera1(fSettings);
  std::cout << "\t-Loaded camera 1" << std::endl;

  // Read second camera if stereo (not rectified)
  if (sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    readCamera2(fSettings);
    std::cout << "\t-Loaded camera 2" << std::endl;
  }

  // Read image info
  readImageInfo(fSettings);
  std::cout << "\t-Loaded image info" << std::endl;

  if (sensor_ == MORB_SLAM::CameraType::IMU_MONOCULAR || sensor_ == MORB_SLAM::CameraType::IMU_STEREO ||
      sensor_ == MORB_SLAM::CameraType::IMU_RGBD) {
    readIMU(fSettings);
    std::cout << "\t-Loaded IMU calibration" << std::endl;
  }

  if (sensor_ == MORB_SLAM::CameraType::RGBD || sensor_ == MORB_SLAM::CameraType::IMU_RGBD) {
    readRGBD(fSettings);
    std::cout << "\t-Loaded RGB-D calibration" << std::endl;
  }

  readORB(fSettings);
  std::cout << "\t-Loaded ORB settings" << std::endl;
  readViewer(fSettings);
  std::cout << "\t-Loaded viewer settings" << std::endl;
  readLoadAndSave(fSettings);
  std::cout << "\t-Loaded Atlas settings" << std::endl;
  readOtherParameters(fSettings);
  std::cout << "\t-Loaded misc parameters" << std::endl;

  if (bNeedToRectify_) {
    precomputeRectificationMaps();
    std::cout << "\t-Computed rectification maps" << std::endl;
  }

  std::cout << "----------------------------------" << std::endl;
}

void Settings::readCamera1(cv::FileStorage& fSettings) {
  bool found;

  // Read camera model
  std::string cameraModel = readParameter<std::string>(fSettings, "Camera.type", found);

  std::vector<float> vCalibration;
  if (cameraModel == "PinHole") {
    cameraType_ = PinHole;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_ = std::make_shared<Pinhole>(vCalibration);
    originalCalib1_ = new Pinhole(vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera1.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera1.k3", found, false);
      if (found) {
        vPinHoleDistorsion1_.resize(5);
        vPinHoleDistorsion1_[4] =
            readParameter<float>(fSettings, "Camera1.k3", found);
      } else {
        vPinHoleDistorsion1_.resize(4);
      }
      vPinHoleDistorsion1_[0] =
          readParameter<float>(fSettings, "Camera1.k1", found);
      vPinHoleDistorsion1_[1] =
          readParameter<float>(fSettings, "Camera1.k2", found);
      vPinHoleDistorsion1_[2] =
          readParameter<float>(fSettings, "Camera1.p1", found);
      vPinHoleDistorsion1_[3] =
          readParameter<float>(fSettings, "Camera1.p2", found);
    }

    // Check if we need to correct distortion from the images
    if ((sensor_ == MORB_SLAM::CameraType::MONOCULAR || sensor_ == MORB_SLAM::CameraType::IMU_MONOCULAR) &&
        vPinHoleDistorsion1_.size() != 0) {
      bNeedToUndistort_ = true;
    }
  } else if (cameraModel == "Rectified") {
    cameraType_ = Rectified;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_ = std::make_shared<Pinhole>(vCalibration);
    originalCalib1_ = new Pinhole(vCalibration);

    // Rectified images are assumed to be ideal PinHole images (no distortion)
  } else if (cameraModel == "KannalaBrandt8") {
    cameraType_ = KannalaBrandt;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    calibration1_ = std::make_shared<KannalaBrandt8>(vCalibration);
    originalCalib1_ = new KannalaBrandt8(vCalibration);

    if (sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
      int colBegin =
          readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
      int colEnd =
          readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
      std::vector<int> vOverlapping = {colBegin, colEnd};

      std::reinterpret_pointer_cast<KannalaBrandt8>(calibration1_)->mvLappingArea = vOverlapping;
    }
  } else {
    std::cerr << "Error: " << cameraModel << " not known" << std::endl;
    throw std::invalid_argument("Error: " + cameraModel + " not known");
  }
}

void Settings::readCamera2(cv::FileStorage& fSettings) {
  bool found;
  std::vector<float> vCalibration;
  if (cameraType_ == PinHole) {
    bNeedToRectify_ = true;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration2_ = std::make_shared<Pinhole>(vCalibration);
    originalCalib2_ = new Pinhole(vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera2.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera2.k3", found, false);
      if (found) {
        vPinHoleDistorsion2_.resize(5);
        vPinHoleDistorsion2_[4] =
            readParameter<float>(fSettings, "Camera2.k3", found);
      } else {
        vPinHoleDistorsion2_.resize(4);
      }
      vPinHoleDistorsion2_[0] =
          readParameter<float>(fSettings, "Camera2.k1", found);
      vPinHoleDistorsion2_[1] =
          readParameter<float>(fSettings, "Camera2.k2", found);
      vPinHoleDistorsion2_[2] =
          readParameter<float>(fSettings, "Camera2.p1", found);
      vPinHoleDistorsion2_[3] =
          readParameter<float>(fSettings, "Camera2.p2", found);
    }
  } else if (cameraType_ == KannalaBrandt) {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    calibration2_ = std::make_shared<KannalaBrandt8>(vCalibration);
    originalCalib2_ = new KannalaBrandt8(vCalibration);

    int colBegin =
        readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
    int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd", found);
    std::vector<int> vOverlapping = {colBegin, colEnd};

    std::reinterpret_pointer_cast<KannalaBrandt8>(calibration2_)->mvLappingArea = vOverlapping;
  }

  // Load stereo extrinsic calibration
  if (cameraType_ == Rectified) {
    b_ = readParameter<float>(fSettings, "Stereo.b", found);
    bf_ = b_ * calibration1_->getParameter(0);
  } else {
    cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
    Tlr_ = Converter::toSophus(cvTlr);

    // TODO: also search for Trl and invert if necessary

    b_ = Tlr_.translation().norm();
    bf_ = b_ * calibration1_->getParameter(0);
  }

  thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
}

void Settings::readImageInfo(cv::FileStorage& fSettings) {
  bool found;
  // Read original and desired image dimensions
  int originalRows = readParameter<int>(fSettings, "Camera.height", found);
  int originalCols = readParameter<int>(fSettings, "Camera.width", found);
  originalImSize_.width = originalCols;
  originalImSize_.height = originalRows;

  newImSize_ = originalImSize_;
  int newHeigh =
      readParameter<int>(fSettings, "Camera.newHeight", found, false);
  if (found) {
    bNeedToResize1_ = true;
    newImSize_.height = newHeigh;

    if (!bNeedToRectify_) {
      // Update calibration
      float scaleRowFactor =
          (float)newImSize_.height / (float)originalImSize_.height;
      calibration1_->setParameter(
          calibration1_->getParameter(1) * scaleRowFactor, 1);
      calibration1_->setParameter(
          calibration1_->getParameter(3) * scaleRowFactor, 3);

      if ((sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) &&
          cameraType_ != Rectified) {
        calibration2_->setParameter(
            calibration2_->getParameter(1) * scaleRowFactor, 1);
        calibration2_->setParameter(
            calibration2_->getParameter(3) * scaleRowFactor, 3);
      }
    }
  }

  int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
  if (found) {
    bNeedToResize1_ = true;
    newImSize_.width = newWidth;

    if (!bNeedToRectify_) {
      // Update calibration
      float scaleColFactor =
          (float)newImSize_.width / (float)originalImSize_.width;
      calibration1_->setParameter(
          calibration1_->getParameter(0) * scaleColFactor, 0);
      calibration1_->setParameter(
          calibration1_->getParameter(2) * scaleColFactor, 2);

      if ((sensor_ == MORB_SLAM::CameraType::STEREO || sensor_ == MORB_SLAM::CameraType::IMU_STEREO) &&
          cameraType_ != Rectified) {
        calibration2_->setParameter(
            calibration2_->getParameter(0) * scaleColFactor, 0);
        calibration2_->setParameter(
            calibration2_->getParameter(2) * scaleColFactor, 2);

        if (cameraType_ == KannalaBrandt) {
          std::reinterpret_pointer_cast<KannalaBrandt8>(calibration1_)->mvLappingArea[0] *=
              scaleColFactor;
          std::reinterpret_pointer_cast<KannalaBrandt8>(calibration1_)->mvLappingArea[1] *=
              scaleColFactor;

          std::reinterpret_pointer_cast<KannalaBrandt8>(calibration2_)->mvLappingArea[0] *=
              scaleColFactor;
          std::reinterpret_pointer_cast<KannalaBrandt8>(calibration2_)->mvLappingArea[1] *=
              scaleColFactor;
        }
      }
    }
  }

  fps_ = readParameter<int>(fSettings, "Camera.fps", found);
}

void Settings::readIMU(cv::FileStorage& fSettings) {
  bool found;
  noiseGyro_ = readParameter<float>(fSettings, "IMU.NoiseGyro", found);
  noiseAcc_ = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
  gyroWalk_ = readParameter<float>(fSettings, "IMU.GyroWalk", found);
  accWalk_ = readParameter<float>(fSettings, "IMU.AccWalk", found);
  imuFrequency_ = readParameter<float>(fSettings, "IMU.Frequency", found);

  cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
  Tbc_ = Converter::toSophus(cvTbc);

  readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
  if (found) {
    insertKFsWhenLost_ = (bool)readParameter<int>(
        fSettings, "IMU.InsertKFsWhenLost", found, false);
  } else {
    insertKFsWhenLost_ = true;
  }
}

void Settings::readRGBD(cv::FileStorage& fSettings) {
  bool found;

  depthMapFactor_ =
      readParameter<float>(fSettings, "RGBD.DepthMapFactor", found);
  thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
  b_ = readParameter<float>(fSettings, "Stereo.b", found);
  bf_ = b_ * calibration1_->getParameter(0);
}

void Settings::readORB(cv::FileStorage& fSettings) {
  bool found;

  nFeatures_ = readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
  scaleFactor_ =
      readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
  nLevels_ = readParameter<int>(fSettings, "ORBextractor.nLevels", found);
  initThFAST_ = readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
  minThFAST_ = readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

void Settings::readViewer(cv::FileStorage& fSettings) {
  bool found;

  keyFrameSize_ = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
  keyFrameLineWidth_ =
      readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
  graphLineWidth_ =
      readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
  pointSize_ = readParameter<float>(fSettings, "Viewer.PointSize", found);
  cameraSize_ = readParameter<float>(fSettings, "Viewer.CameraSize", found);
  cameraLineWidth_ =
      readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
  viewPointX_ = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
  viewPointY_ = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
  viewPointZ_ = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
  viewPointF_ = readParameter<float>(fSettings, "Viewer.ViewpointF", found);
  imageViewerScale_ =
      readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

  if (!found) imageViewerScale_ = 1.0f;
}

void Settings::readLoadAndSave(cv::FileStorage& fSettings) {
  bool found;

  sLoadFrom_ = readParameter<std::string>(fSettings, "System.LoadAtlasFromFile",
                                     found, false);
  sSaveto_ =
      readParameter<std::string>(fSettings, "System.SaveAtlasToFile", found, false);
}

void Settings::readOtherParameters(cv::FileStorage& fSettings) {
  bool found;

  thFarPoints_ =
      readParameter<float>(fSettings, "System.thFarPoints", found, false);
}

void Settings::precomputeRectificationMaps() {
  // Precompute rectification maps, new calibrations, ...
  cv::Mat K1 = std::reinterpret_pointer_cast<Pinhole>(calibration1_)->toK();
  K1.convertTo(K1, CV_64F);
  cv::Mat K2 = std::reinterpret_pointer_cast<Pinhole>(calibration2_)->toK();
  K2.convertTo(K2, CV_64F);

  cv::Mat cvTlr;
  cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
  cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
  R12.convertTo(R12, CV_64F);
  cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
  t12.convertTo(t12, CV_64F);

  cv::Mat R_r1_u1, R_r2_u2;
  cv::Mat P1, P2, Q;

  cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(),
                    newImSize_, R12, t12, R_r1_u1, R_r2_u2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, -1, newImSize_);
  cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1,
                              P1.rowRange(0, 3).colRange(0, 3), newImSize_,
                              CV_32F, M1l_, M2l_);
  cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2,
                              P2.rowRange(0, 3).colRange(0, 3), newImSize_,
                              CV_32F, M1r_, M2r_);

  // Update calibration
  calibration1_->setParameter(P1.at<double>(0, 0), 0);
  calibration1_->setParameter(P1.at<double>(1, 1), 1);
  calibration1_->setParameter(P1.at<double>(0, 2), 2);
  calibration1_->setParameter(P1.at<double>(1, 2), 3);

  // Update bf
  bf_ = b_ * P1.at<double>(0, 0);

  // Update relative pose between camera 1 and IMU if necessary
  if (sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    Eigen::Matrix3f eigenR_r1_u1;
    cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
    Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
    Tbc_ = Tbc_ * T_r1_u1.inverse();
  }
}

std::ostream& operator<<(std::ostream& output, const Settings& settings) {
  output << "SLAM settings: " << std::endl;

  output << "\t-Camera 1 parameters (";
  if (settings.cameraType_ == Settings::PinHole ||
      settings.cameraType_ == Settings::Rectified) {
    output << "Pinhole";
  } else {
    output << "Kannala-Brandt";
  }
  output << ")"
         << ": [";
  for (size_t i = 0; i < settings.originalCalib1_->size(); i++) {
    output << " " << settings.originalCalib1_->getParameter(i);
  }
  output << " ]" << std::endl;

  if (!settings.vPinHoleDistorsion1_.empty()) {
    output << "\t-Camera 1 distortion parameters: [ ";
    for (float d : settings.vPinHoleDistorsion1_) {
      output << " " << d;
    }
    output << " ]" << std::endl;
  }
  std::cout << "bout to start camera 2 stuff\n";
  if (settings.sensor_ == MORB_SLAM::CameraType::STEREO ||
      settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    output << "\t-Camera 2 parameters (";
    if (settings.cameraType_ == Settings::PinHole ||
        settings.cameraType_ == Settings::Rectified) {
      output << "Pinhole";
    } else {
      output << "Kannala-Brandt";
    }
    output << ""
           << ": [";
    for (size_t i = 0; i < settings.originalCalib2_->size(); i++) {
      output << " " << settings.originalCalib2_->getParameter(i);
    }
    output << " ]" << std::endl;

    if (!settings.vPinHoleDistorsion2_.empty()) {
      output << "\t-Camera 1 distortion parameters: [ ";
      for (float d : settings.vPinHoleDistorsion2_) {
        output << " " << d;
      }
      output << " ]" << std::endl;
    }
  }

  output << "\t-Original image size: [ " << settings.originalImSize_.width
         << " , " << settings.originalImSize_.height << " ]" << std::endl;
  output << "\t-Current image size: [ " << settings.newImSize_.width << " , "
         << settings.newImSize_.height << " ]" << std::endl;

  if (settings.bNeedToRectify_) {
    output << "\t-Camera 1 parameters after rectification: [ ";
    for (size_t i = 0; i < settings.calibration1_->size(); i++) {
      output << " " << settings.calibration1_->getParameter(i);
    }
    output << " ]" << std::endl;
  } else if (settings.bNeedToResize1_) {
    output << "\t-Camera 1 parameters after resize: [ ";
    for (size_t i = 0; i < settings.calibration1_->size(); i++) {
      output << " " << settings.calibration1_->getParameter(i);
    }
    output << " ]" << std::endl;

    if ((settings.sensor_ == MORB_SLAM::CameraType::STEREO ||
         settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO) &&
        settings.cameraType_ == Settings::KannalaBrandt) {
      output << "\t-Camera 2 parameters after resize: [ ";
      for (size_t i = 0; i < settings.calibration2_->size(); i++) {
        output << " " << settings.calibration2_->getParameter(i);
      }
      output << " ]" << std::endl;
    }
  }

  output << "\t-Sequence FPS: " << settings.fps_ << std::endl;

  // Stereo stuff
  if (settings.sensor_ == MORB_SLAM::CameraType::STEREO ||
      settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO) {
    output << "\t-Stereo baseline: " << settings.b_ << std::endl;
    output << "\t-Stereo depth threshold : " << settings.thDepth_ << std::endl;

    if (settings.cameraType_ == Settings::KannalaBrandt) {
      auto vOverlapping1 =
          std::reinterpret_pointer_cast<KannalaBrandt8>(settings.calibration1_)->mvLappingArea;
      auto vOverlapping2 =
          std::reinterpret_pointer_cast<KannalaBrandt8>(settings.calibration2_)->mvLappingArea;
      output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0] << " , "
             << vOverlapping1[1] << " ]" << std::endl;
      output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0] << " , "
             << vOverlapping2[1] << " ]" << std::endl;
    }
  }

  if (settings.sensor_ == MORB_SLAM::CameraType::IMU_MONOCULAR ||
      settings.sensor_ == MORB_SLAM::CameraType::IMU_STEREO ||
      settings.sensor_ == MORB_SLAM::CameraType::IMU_RGBD) {
    output << "\t-Gyro noise: " << settings.noiseGyro_ << std::endl;
    output << "\t-Accelerometer noise: " << settings.noiseAcc_ << std::endl;
    output << "\t-Gyro walk: " << settings.gyroWalk_ << std::endl;
    output << "\t-Accelerometer walk: " << settings.accWalk_ << std::endl;
    output << "\t-IMU frequency: " << settings.imuFrequency_ << std::endl;
  }

  if (settings.sensor_ == MORB_SLAM::CameraType::RGBD ||
      settings.sensor_ == MORB_SLAM::CameraType::IMU_RGBD) {
    output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_ << std::endl;
  }

  output << "\t-Features per image: " << settings.nFeatures_ << std::endl;
  output << "\t-ORB scale factor: " << settings.scaleFactor_ << std::endl;
  output << "\t-ORB number of scales: " << settings.nLevels_ << std::endl;
  output << "\t-Initial FAST threshold: " << settings.initThFAST_ << std::endl;
  output << "\t-Min FAST threshold: " << settings.minThFAST_ << std::endl;

  return output;
}
};  // namespace MORB_SLAM
