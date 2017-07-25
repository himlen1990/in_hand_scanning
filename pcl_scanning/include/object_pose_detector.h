#ifndef OBJECT_POSE_DETECTOR_H
#define OBJECT_POSE_DETECTOR_H

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class object_pose_detector
{
 private:
  cv::Mat rgbImg_;
  cv::Mat depth_;
  float lowerb_;
  float upperb_;
  cv::Point thumb_;
  cv::Point forefinger_;


 public:
  object_pose_detector(const cv::Mat RGBImg, const cv::Mat depth, const float lowerb, const float upperb);
  bool detect_pose(std::vector<cv::Point> finger_positions);
};

#endif
