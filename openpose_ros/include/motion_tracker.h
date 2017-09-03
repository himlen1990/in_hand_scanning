#ifndef MOTION_TRACKER_H
#define MOTION_TRACKER_H

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
#include <eigen3/Eigen/Dense>
#include "hand_keypoint.h"
#include "keyframe.h"

#include <fstream>
using namespace std;
using namespace cv;


class motion_tracker
{
 private:

  std::vector<keyframe> keyframes_;
  keyframe model_frame_;
  cv::Mat camera_matrix_;
  float rotated_angle_;
  int reset_frame_index_;
  float rotated_angle_in_one_grasp_;

  float accumulate_angle_;
  int counter;

 public:
  motion_tracker();
  bool add_keyframe(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in);
  float estimate_rotation(keyframe model_frame, std::vector<hand_keypoint> keypoint2D);
  void reset_model_frame();

  Eigen::Matrix4f set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix);
};

#endif
