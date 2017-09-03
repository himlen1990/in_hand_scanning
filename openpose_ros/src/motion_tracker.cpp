#include "motion_tracker.h"
#include <iostream>
//get finger color in init func, then detect that color or directly use kcf
//1.give a bounding box of two fingers, after confirm(press a key), start tracking

motion_tracker::motion_tracker()
{
  rotated_angle_ = 0;
  reset_frame_index_ = 0;
  rotated_angle_in_one_grasp_ = 0;
  accumulate_angle_ = 0;
  counter = 0;

}

bool motion_tracker::add_keyframe(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in)
{

  int valid_point_counter = 0;
  float pre_z = 0;
  bool set_z0 = false;
  char pcd_file[100];
  char text_file[100];

  if(hand_keypoints.size() > 0)
    for(int i=0; i<hand_keypoints.size(); i++)
      {
	if(hand_keypoints[i].get_score() > 0.4)	
	  {
	    float z = hand_keypoints[i].get_xyz()(2);
	    if(!set_z0)
	      {
		pre_z = z;
		set_z0 = true;
	      }
	    if(z>0 && abs(z-pre_z)<0.3)
	      {
		valid_point_counter++;
	      }	    
	    pre_z = z;
	  }
      }
  if (valid_point_counter > 1)
    {
      cv::Mat keyframe_rgb = rgb.clone();
      cv::Mat keyframe_depth = depth.clone();    
      float rotated_angle = 0;
      cout<<"size "<<keyframes_.size()<<endl;
      if(model_frame_.is_empty())
	{
	  keyframe kf;
	  kf.setup(rgb,depth,hand_keypoints,cloud_in);      
	  keyframes_.push_back(kf);
	  model_frame_ = kf;
	  snprintf(pcd_file,100,"data_%02d.pcd",counter);
	  snprintf(text_file,100,"tran_%02d.txt",counter);
	  pcl::io::savePCDFileASCII(pcd_file,*cloud_in);
	  ofstream ofs(text_file, fstream::out);  
	  ofs<<kf.get_central_point();
	  ofs<<endl;
	  ofs<<accumulate_angle_;
	  ofs.close(); 
	  counter++;
	  return true;
	}      
      else
	{
	  rotated_angle = estimate_rotation(model_frame_,hand_keypoints);
	  cout<<rotated_angle<<endl;	  

	  rotated_angle_in_one_grasp_ = estimate_rotation(keyframes_[reset_frame_index_],hand_keypoints); 
	  float total_angle = rotated_angle_ + rotated_angle_in_one_grasp_;
	  cout<<"total "<<total_angle<<endl;
	  cout<<"accumulated "<<accumulate_angle_<<endl;
	}
      if(rotated_angle > 3 && rotated_angle < 20)
	{
	  accumulate_angle_ = accumulate_angle_ + rotated_angle;
	  keyframe kf;
	  kf.setup(rgb,depth,hand_keypoints,cloud_in);
	  kf.set_pose(accumulate_angle_);
	  snprintf(pcd_file,100,"data_%02d.pcd",counter);
	  snprintf(text_file,100,"tran_%02d.txt",counter);
	  pcl::io::savePCDFileASCII(pcd_file,*(kf.get_cloud()));
	  ofstream ofs(text_file, fstream::out);  
	  ofs<<kf.get_central_point();
	  ofs<<endl;
	  ofs<<accumulate_angle_;
	  ofs.close(); 
	  counter++;

	  //cout<<"!!!!!!!!"<<endl;
	  //cout<<"central_point"<<kf.get_central_point()<<endl;
	  keyframes_.push_back(kf);
	  model_frame_ = kf;
	  return true;

	}
    }

  return false;
  
}




float motion_tracker::estimate_rotation(keyframe model_frame, std::vector<hand_keypoint> hand_keypoints)
{
  float angle = 0;

  Eigen::Matrix4f P_matrix = Eigen::Matrix4f::Zero();
  Eigen::Vector3f p1 = model_frame.get_keypoints()[0].get_xyz();
  Eigen::Vector3f p2 = model_frame.get_keypoints()[1].get_xyz();

  if(hand_keypoints.size() > 0)
    {
      Eigen::Vector3f p3 = hand_keypoints[0].get_xyz(); 
      Eigen::Vector3f p4 = hand_keypoints[1].get_xyz();
      
      Eigen::Vector3f v1 = p2 - p1;
      Eigen::Vector3f v2 = p4 - p3;

      Eigen::Vector2f v1_noy(v1(0),v1(2));
      Eigen::Vector2f v2_noy(v2(0),v2(2));

      
      Eigen::Vector2f v1_norm = v1_noy.normalized();
      Eigen::Vector2f v2_norm = v2_noy.normalized();
      float theta = acos(v1_norm.dot(v2_norm));
      angle = theta * 57.2958;
    }
  return angle;
}

void motion_tracker::reset_model_frame()
{
  keyframe empty_frame;
  model_frame_ = empty_frame;
  reset_frame_index_ = keyframes_.size();
  rotated_angle_ = rotated_angle_ + rotated_angle_in_one_grasp_;
  rotated_angle_in_one_grasp_ = 0;
}

Eigen::Matrix4f motion_tracker::set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix)
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
