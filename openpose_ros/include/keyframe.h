#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "hand_keypoint.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

class keyframe
{
 private:
  std::vector<hand_keypoint> keypoints_;
  Eigen::Matrix4f world_pose;
  cv::Mat rgb_;//for debug
  cv::Mat depth_;//for debug
  Eigen::Vector3f central_point_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  bool empty_;
  float world_pose_;
 public:
 keyframe():empty_(true),cloud_(new pcl::PointCloud<pcl::PointXYZRGB>())
    {}

  void setup(cv::Mat rgb, cv::Mat depth,std::vector<hand_keypoint> kp, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in)
  {
    keypoints_ = kp;
    rgb_ = rgb;
    depth_ = depth;
    empty_ = false;
    pcl::copyPointCloud(*cloud_in, *cloud_);
    compute_central_point();
  }

  bool is_empty()
  {
    return empty_;
  }

  void compute_central_point()
    {
      float x = (keypoints_[1].get_xyz()(0) + keypoints_[0].get_xyz()(0))/2;
      float y = (keypoints_[1].get_xyz()(1) + keypoints_[0].get_xyz()(1))/2;
      float z = (keypoints_[1].get_xyz()(2) + keypoints_[0].get_xyz()(2))/2;
      central_point_ = Eigen::Vector3f(x,y,z);
    }
  Eigen::Vector3f get_central_point()
  {
    return central_point_;
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
  
  void set_pose(float pose)
  {
    world_pose_ = pose;
  }
  
  float get_pose()
  {
    return world_pose_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud()
    {
      return cloud_;
    }

};


#endif
