#ifndef CAM_MOTION_TRACKER_H
#define CAM_MOTION_TRACKER_H

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
//#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace cv;

class point_with_score
{
 public:
  cv::Point2f pt;
  float score;
  

  point_with_score(cv::Point2f pti,float scorei)
   {
     pt = pti;
     score = scorei;
   }
};

class cam_motion_tracker
{
 private:

  cv::Mat first_frame_rgb_;
  cv::Mat first_frame_depth_;
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
  
  float median;
  std::vector<float> dis;

  cv::Mat camera_matrix_;
  
  std::vector<point_with_score> hand_keypoints_model_;
  
 public:
  cam_motion_tracker();
  bool init(cv::Mat rgb, cv::Mat depth, std::vector<point_with_score> keypoint2D);
  Eigen::Matrix4f regist(cv::Mat rgb, cv::Mat depth, std::vector<point_with_score> keypoint2D);
  vector<Point2f> Points(vector<KeyPoint> keypoints);
  cv::Point3f unproject(cv::Point kp ,const float z);

  void set_camera_info(const boost::array<double,9> camera_info);
  Eigen::Matrix4f set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix);
};

#endif
