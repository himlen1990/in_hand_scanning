#ifndef CAM_MOTION_TRACKER_H
#define CAM_MOTION_TRACKER_H

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
#include <eigen3/Eigen/Dense>
#include "hand_keypoint.h"
#include "keyframe.h"
using namespace std;
using namespace cv;


class cam_motion_tracker
{
 private:

  std::vector<keyframe> keyframe_pair_;

  cv::Mat first_frame_rgb_;
  cv::Mat first_frame_depth_;
  cv::Mat second_frame_;

  cv::Point3f p1,med;
  
  
  bool test_flag;



  std::vector<float> dis;

  cv::Mat camera_matrix_;
  
  
 public:
  cam_motion_tracker();
  bool add_keyframe(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> keypoint2D);
  Eigen::Matrix4f regist(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> keypoint2D);




  Eigen::Matrix4f set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix);
};

#endif
