#ifndef HAND_KEYPOINT_H
#define HAND_KEYPOINT_H

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>

class hand_keypoint
{
 private:
  Eigen::Vector3f xyz_;
  cv::Point2f uv_;
  float score_;

 public:
  
 hand_keypoint(cv::Point2f kp, float score):uv_(kp),score_(score)
  {}
  
  void set_xyz(Eigen::Vector3f xyz)
  {
    xyz_ = xyz;
  }
  
  Eigen::Vector3f get_xyz()
    {   
      return xyz_;
    }
  
  cv::Point2f get_uv()
    {     
      return uv_;
    }

  float get_score()
  {
    return score_;
  }
};


#endif
