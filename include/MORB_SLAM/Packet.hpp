#pragma once
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <optional>


namespace MORB_SLAM{

struct Packet{
    std::optional<Sophus::SE3f> pose;

    Packet();
    Packet(const Sophus::SE3f &pose);
    virtual ~Packet();
};
struct InertialPacket{
};

struct StereoPacket : public Packet, public InertialPacket {
    cv::Mat imgLeft;
    cv::Mat imgRight;
    StereoPacket(const cv::Mat &imgLeft, const cv::Mat &imgRight);
    StereoPacket(const Sophus::SE3f &pose, const cv::Mat &imgLeft, const cv::Mat &imgRight);
};

struct MonoPacket : public Packet, public InertialPacket {
    cv::Mat img;
    MonoPacket(const cv::Mat &img);
    MonoPacket(const Sophus::SE3f &pose, const cv::Mat &img);
};

struct RGBDPacket : public Packet, public InertialPacket {
    cv::Mat img;
    cv::Mat depthImg;
    RGBDPacket(const cv::Mat &img, const cv::Mat &depthImg);
    RGBDPacket(const Sophus::SE3f &pose, const cv::Mat &img, const cv::Mat &depthImg);
};
}