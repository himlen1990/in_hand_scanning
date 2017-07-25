#ifndef FINER_POSITION_TRACKER_H
#define FINER_POSITION_TRACKER_H

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
//#include <openpose>

class finger_position_tracker
{
 private:
  cv::Point thumb_;
  cv::Point forefinger_;
  cv::Rect2d thumb_bbox_;
  cv::Rect2d forefinger_bbox_;
  cv::MultiTracker trackers;
  bool init_flag;

 public:
  finger_position_tracker();
  void start(cv::Mat frame, cv::Point thumb, cv::Point forefinger, int bbox_width, int bbox_height);
  std::vector<cv::Point> update(cv::Mat frame);
  std::vector<cv::Point> update_using_new_detect_result(cv::Mat frame, cv::Point thumb, cv::Point forefinger);
};

#endif
