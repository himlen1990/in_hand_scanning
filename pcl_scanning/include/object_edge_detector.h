#ifndef OBJECT_EDGE_DETECTOR_H
#define OBJECT_EDGE_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class object_edge_detector
{
 private:
  cv::Mat depth_;
  float lowerb_;
  float upperb_;
  
 public:
  object_edge_detector(const cv::Mat depth, const float lowerb, const float upperb);
  bool detect_edge();
};

#endif
