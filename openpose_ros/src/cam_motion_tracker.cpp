#include "cam_motion_tracker.h"
#include <iostream>
//get finger color in init func, then detect that color or directly use kcf
//1.give a bounding box of two fingers, after confirm(press a key), start tracking

cam_motion_tracker::cam_motion_tracker()
{
  test_flag = false;
}

bool cam_motion_tracker::add_keyframe(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints)
{

  int vaild_point_counter = 0;
  float pre_z = 0;
  bool set_z0 = false;
  if(hand_keypoints.size() > 0)
    for(int i=0; i<hand_keypoints.size(); i++)
      {
	if(hand_keypoints[i].get_score() > 0.6)	
	  {
	    float z = hand_keypoints[i].get_xyz()(2);
	    if(!set_z0)
	      {
		pre_z = z;
		set_z0 = true;
	      }
	    if(z>0 && abs(z-pre_z)<0.3)
	      {
		vaild_point_counter++;
	      }	    
	    pre_z = z;
	  }
      }
  
  if (vaild_point_counter > 1)
    {
      cv::Mat keyframe_rgb = rgb.clone();
      cv::Mat keyframe_depth = depth.clone();    
      keyframe kf(rgb,depth,hand_keypoints);
      keyframe_pair_.push_back(kf);
      return true;
    }

  return false;
  
}




Eigen::Matrix4f cam_motion_tracker::regist(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints)
{

  

  Eigen::Matrix4f P_matrix = Eigen::Matrix4f::Zero();
  Eigen::Vector3f p1 = keyframe_pair_[0].get_keypoints()[0].get_xyz();
  Eigen::Vector3f p2 = keyframe_pair_[0].get_keypoints()[1].get_xyz();

  if(hand_keypoints.size() > 0)
    {
      Eigen::Vector3f p3 = hand_keypoints[0].get_xyz(); 
      Eigen::Vector3f p4 = hand_keypoints[1].get_xyz();
      
      Eigen::Vector3f v1 = p2 - p1;
      Eigen::Vector3f v2 = p4 - p3;
      
      Eigen::Vector3f v1_norm = v1.normalized();
      Eigen::Vector3f v2_norm = v2.normalized();
      float angle = acos(v1_norm.dot(v2_norm));
      //cout<<angle*57.3<<endl;
    }
      return P_matrix;
}


Eigen::Matrix4f cam_motion_tracker::set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
  // Rotation-Translation Matrix Definition
  Eigen::Matrix4f _P_matrix = Eigen::Matrix4f::Zero();
  _P_matrix<< 
  R_matrix.at<double>(0,0),R_matrix.at<double>(0,1),R_matrix.at<double>(0,2),t_matrix.at<double>(0),
  R_matrix.at<double>(1,0),R_matrix.at<double>(1,1),R_matrix.at<double>(1,2),t_matrix.at<double>(1),
  R_matrix.at<double>(2,0),R_matrix.at<double>(2,1),R_matrix.at<double>(2,2),t_matrix.at<double>(2),
    0, 0, 0, 1;
  return _P_matrix;
}
