#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "hand_keypoint.h"
class keyframe
{
 private:
  std::vector<hand_keypoint> keypoints_;
  Eigen::Matrix4f world_pose;
  cv::Mat rgb_;//for debug
  cv::Mat depth_;//for debug
  cv::Point3f median_point;

 public:
 keyframe(cv::Mat rgb, cv::Mat depth,std::vector<hand_keypoint> kp):keypoints_(kp),rgb_(rgb),depth_(depth)
    {}
  void compute_median_point()
    {
      float x = (keypoints_[1].get_xyz()(0) + keypoints_[0].get_xyz()(0))/2;
      float y = (keypoints_[1].get_xyz()(1) + keypoints_[0].get_xyz()(1))/2;
      float z = (keypoints_[1].get_xyz()(2) + keypoints_[0].get_xyz()(2))/2;
    }
  cv::Point3f get_median_point()
  {
    return median_point;
  }

  std::vector<hand_keypoint> get_keypoints()
    {
      return keypoints_;
    }
  
  cv::Mat get_rgb()
    {
      return rgb_;
    }

  cv::Mat get_depth()
    {
      return depth_;
    }
  
};


#endif
