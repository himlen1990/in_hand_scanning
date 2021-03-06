#ifndef OBJECT_POSE_TRACKER_H
#define OBJECT_POSE_TRACKER_H

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/array.hpp>
#include <Eigen/Dense>

class object_pose_tracker
{
 private:
  cv::Mat rgbImg_;
  cv::Mat depth_;
  float lowerb_;
  float upperb_;
  cv::Point thumb_;
  cv::Point forefinger_;
  boost::array<double,9> camera_info_k_;
  //std::vector<cv::Point3f> pose3D_;
  Eigen::Matrix4f rotM_;
  cv::Point3f orig_;

 public:
  object_pose_tracker(const cv::Mat RGBImg, const cv::Mat depth, const float lowerb, const float upperb, const boost::array<double,9> camera_info_k);
  bool init_pose(std::vector<cv::Point2f> finger_positions);
  Eigen::Matrix4f get_3D_pose();
  cv::Point3f get_t();
};

#endif
