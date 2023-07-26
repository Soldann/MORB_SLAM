#include "MORB_SLAM/Packet.hpp"



namespace MORB_SLAM{

Packet::Packet() {} // optional initialized to empty
Packet::Packet(const Sophus::SE3f &pose) : pose{pose} {}
Packet::~Packet(){}


StereoPacket::StereoPacket(const cv::Mat &imgLeft, const cv::Mat &imgRight):
    Packet(), imgLeft{imgLeft}, imgRight{imgRight} {}
StereoPacket::StereoPacket(const Sophus::SE3f &pose, const cv::Mat &imgLeft, const cv::Mat &imgRight):
    Packet(pose), imgLeft{imgLeft}, imgRight{imgRight} {}

MonoPacket::MonoPacket(const cv::Mat &img):
    Packet(), img{img} {}
MonoPacket::MonoPacket(const Sophus::SE3f &pose, const cv::Mat &img):
    Packet(pose), img{img} {}

RGBDPacket::RGBDPacket(const cv::Mat &img, const cv::Mat &depthImg):
    Packet(), img{img}, depthImg{depthImg} {}
RGBDPacket::RGBDPacket(const Sophus::SE3f &pose, const cv::Mat &img, const cv::Mat &depthImg):
    Packet(pose), img{img}, depthImg{depthImg} {}


}
