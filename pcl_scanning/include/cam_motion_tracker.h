#ifndef CAM_MOTION_TRACKER_H
#define CAM_MOTION_TRACKER_H

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace cv;


class cam_motion_tracker
{
 private:
  Ptr<ORB> orb;
  cv::Mat first_frame_;
  cv::Mat second_frame_;
  cv::Mat desc1_;
  cv::Mat desc2_;
  vector<cv::KeyPoint> kpts1_;
  vector<cv::KeyPoint> kpts2_;
  float inlier_threshold_;  // Distance threshold to identify inliers  
  float nn_match_ratio_;   // Nearest neighbor matching ratio  
  double ransac_thresh_; // RANSAC inlier threshold 
  cv::Rect roi_;
  
  float fx;
  float fy;
  float cx;
  float cy;
  float invfx;
  float invfy;
  
  cv::Mat camera_matrix_;
  
 public:
  cam_motion_tracker();
  void start(cv::Mat frame, cv::Rect roi);
  Eigen::Matrix4f update(cv::Mat rgb, cv::Mat depth);
  vector<Point2f> Points(vector<KeyPoint> keypoints);
  cv::Point3f unproject(cv::KeyPoint kp ,const float z);
  void set_camera_info(const boost::array<double,9> camera_info);
  Eigen::Matrix4f set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix);
};

#endif
