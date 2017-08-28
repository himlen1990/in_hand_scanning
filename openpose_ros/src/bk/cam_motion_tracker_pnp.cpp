#include "cam_motion_tracker.h"
#include <iostream>
//get finger color in init func, then detect that color or directly use kcf
//1.give a bounding box of two fingers, after confirm(press a key), start tracking

cam_motion_tracker::cam_motion_tracker()
{}

bool cam_motion_tracker::init(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints)
{
  cout<<"wait for init"<<endl;
  int vaild_point_counter = 0;
  if(hand_keypoints.size() > 0)
    for(int i=0; i<hand_keypoints.size(); i++)
      if(hand_keypoints[i].get_uv().x > 1.0)
	vaild_point_counter++;
  if (vaild_point_counter > 6)
    {
      hand_keypoints_model_ = hand_keypoints;
      cout<<"init finish"<<hand_keypoints_model_.size()<<endl;
      cv::Mat keyframe_rgb = rgb.clone();
      cv::Mat keyframe_depth = depth.clone();    
      add_keyframe(keyframe_rgb,keyframe_depth,hand_keypoints);

      return true;
    }
  else 
    return false;
  
}

void cam_motion_tracker::add_keyframe(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints)
{
  for(int i=0; i<hand_keypoints.size(); i++)
    {
      float z =  depth.at<float>(hand_keypoints[i].get_uv().y, hand_keypoints[i].get_uv().x);	  
      cv::Point3f Pos = unproject(hand_keypoints[i].get_uv(),z);
      hand_keypoints[i].set_xyz(Pos);	  
      
    }
  keyframe kf(rgb,depth,hand_keypoints);
  kf.compute_median();
  keyframes_.push_back(kf);
  
}


cv::Point3f cam_motion_tracker::unproject(cv::Point kp, const float z)
{
  const float u = kp.x;
  const float v = kp.y;
  const float x = (u-cx)*z*invfx;
  const float y = (v-cy)*z*invfy;
  cv::Point3f kp3D(x,y,z);
  return kp3D;
}



Eigen::Matrix4f cam_motion_tracker::regist(cv::Mat rgb, cv::Mat depth, std::vector<hand_keypoint> hand_keypoints)
{


  Eigen::Matrix4f P_matrix = Eigen::Matrix4f::Zero();


  vector<cv::Point2f> image_points;
  vector<cv::Point3f> model_points;

  vector<cv::Point2f> homo_points;
  //cout<<hand_keypoints_model_.size()<<endl;
  //for(int i=0 ; i < hand_keypoints.size(); i++)
  //cout<<hand_keypoints[i]<<endl;

  std::vector<cv::KeyPoint> kp1v,kp2v;
  cv::KeyPoint kp;
  std::vector<DMatch> inlier_matches;

  cv::Point3f model_point_5 = cv::Point3f(0,0,0);
  cv::Point3f model_point_4 = cv::Point3f(0,0,0);

#if 1

  keyframe first_kf = keyframes_[0];
  float median = first_kf.get_median();
  cout<<"!!!!"<<endl;
  //cout<<hand_keypoints_model_.size()<<endl;
  //cout<<hand_keypoints.size()<<endl;
  for(int i=0 ; i < hand_keypoints.size(); i++)
    if(first_kf.get_keypoints()[i].get_uv().x !=0 && hand_keypoints[i].get_uv().x !=0 )
      {
	float z = first_kf.get_keypoints()[i].get_xyz().z;

	if (z>0 && abs(z-median)<0.1 && hand_keypoints[i].get_score() > 0.6 && first_kf.get_keypoints()[i].get_score() > 0.6 )
	  {
	    cv::Point3f Pos = first_kf.get_keypoints()[i].get_xyz();
	    model_points.push_back(Pos);	    
	    image_points.push_back(hand_keypoints[i].get_uv());
	    homo_points.push_back(hand_keypoints_model_[i].get_uv());

	    //cout<<model_points<<endl;
	    //cout<<image_points<<endl;

	    //--------draw keypoint-----
	    int new_i = static_cast<int>(kp1v.size());
	    kp.pt = hand_keypoints_model_[i].get_uv();
	    kp1v.push_back(kp);
	    kp.pt = hand_keypoints[i].get_uv();
	    kp2v.push_back(kp);
	    inlier_matches.push_back(DMatch(new_i, new_i, 0));


	    //find axis---------------------------------------
	    if (i == 5)	      
		model_point_5 = Pos;	      
	    if (i == 4)	      
		model_point_4 = Pos;	      
	    
	  }
      }
#endif  
  cout<<"model point size  "<<model_points.size()<<endl;
  if(model_points.size()>6)// slove pnp
    {
      cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
      cv::solvePnPRansac(model_points,image_points,camera_matrix_,distCoeffs,rvec,tvec,false, 30, 3, 6);
      //bool correspondence = cv::solvePnP(model_points3,image_points3,camera_matrix_,distCoeffs,rvec,tvec,false);


      cv::Mat _R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
      Rodrigues(rvec,_R_matrix);
      cv::Mat _t_matrix = tvec;

      std::vector<cv::Point2f> projectedPoints;
      std::vector<cv::Point3f> axis;
      //--------------------find axis----------------------
      float vx = hand_keypoints_model_[5].get_uv().x - hand_keypoints_model_[4].get_uv().x;
      float vy = hand_keypoints_model_[5].get_uv().y - hand_keypoints_model_[4].get_uv().y;
      float mag = sqrt(vx*vx + vy*vy);
      vx = vx/mag;
      vy = vy/mag;
      float temp = vx;
      vx = -vy;
      vy =temp;
      float px = hand_keypoints_model_[4].get_uv().x-vx*5;
      float py = hand_keypoints_model_[4].get_uv().y-vy*5;

      //cv::Point3f vxf =  unproject(hand_keypoints_model_[9].pt,dis[9]) -  unproject(hand_keypoints_model_[8].pt,dis[8]);
      
      cv::Mat keyframe_rgb = first_kf.get_rgb();
      cv::Mat keyframe_depth = first_kf.get_depth();

      cv::line(keyframe_rgb,hand_keypoints_model_[4].get_uv(),hand_keypoints_model_[5].get_uv(),cv::Scalar(0,0,255),2);
      cv::line(keyframe_rgb,hand_keypoints_model_[4].get_uv(),cv::Point(px,py),cv::Scalar(0,0,255),2);

      if (model_point_4.z > 0 && model_point_5.z > 0 && abs(model_point_4.z-median)<0.1  && abs(model_point_5.z-median)<0.1) 
	{
	  cout<<"draw axis"<<endl;
	  Eigen::Vector3f vx3f(model_point_5.x - model_point_4.x, model_point_5.y - model_point_4.y, model_point_5.z - model_point_4.z);
	  float z = keyframe_depth.at<float>(py,px);
	  cv::Point3f ppos = unproject(cv::Point2f(px,py),z);
	  Eigen::Vector3f vy3f(ppos.x - model_point_4.x, ppos.y - model_point_4.y, ppos.z - model_point_4.z);
	  vx3f.normalize();
	  vy3f.normalize();
	  Eigen::Vector3f vz3f = vx3f.cross(vy3f);
	  axis.push_back(cv::Point3f(vx3f(0), vx3f(1), vx3f(2)));
	  axis.push_back(cv::Point3f(vy3f(0), vy3f(1), vy3f(2)));
	  axis.push_back(cv::Point3f(vz3f(0), vz3f(1), vz3f(2)));
	  cv::projectPoints(axis, rvec, tvec, camera_matrix_, distCoeffs, projectedPoints);

	  cv::line(rgb,hand_keypoints[4].get_uv(),projectedPoints[0],cv::Scalar(0,0,255),2);
	  cv::line(rgb,hand_keypoints[4].get_uv(),projectedPoints[1],cv::Scalar(0,255,0),2);
	  cv::line(rgb,hand_keypoints[4].get_uv(),projectedPoints[2],cv::Scalar(255,0,0),2);

	}

      float sy = sqrt(_R_matrix.at<double>(0,0) * _R_matrix.at<double>(0,0) +  _R_matrix.at<double>(1,0) * _R_matrix.at<double>(1,0) );
 
      bool singular = sy < 1e-6; // If
 
      float x=0;
      float y=0;
      float z=0;
      if (!singular)
	{
	  x = atan2(_R_matrix.at<double>(2,1) , _R_matrix.at<double>(2,2));
	  y = atan2(-_R_matrix.at<double>(2,0), sy);
	  z = atan2(_R_matrix.at<double>(1,0), _R_matrix.at<double>(0,0));
	}
      else
	{
	  x = atan2(-_R_matrix.at<double>(1,2), _R_matrix.at<double>(1,1));
	  y = atan2(-_R_matrix.at<double>(2,0), sy);
	  z = 0;
	}
      
      cout<<"x "<<x*57.29<<endl;
      cout<<"y "<<y*57.29<<endl;
      cout<<"z "<<z*57.29<<endl;
      
      P_matrix = set_P_matrix(_R_matrix,_t_matrix);

      //------------------draw keypoints-------
      Mat res;
      drawMatches(keyframe_rgb, kp1v, rgb, kp2v,inlier_matches, res);
      imshow("res.png", res);

      
    }
      

      //second_frame_.copyTo(first_frame_);
      
      return P_matrix;
}


void cam_motion_tracker::set_camera_info(const boost::array<double,9> camera_info)
{
  cv::Mat camera_m = cv::Mat::zeros(3, 3, CV_64FC1);
  fx = camera_info[0];
  fy = camera_info[4];
  cx = camera_info[2];
  cy = camera_info[5];
  invfx = 1.0f/fx;
  invfy = 1.0f/fy;

  camera_m.at<double>(0,0) = fx;
  camera_m.at<double>(1,1) = fy;
  camera_m.at<double>(0,2) = cx;
  camera_m.at<double>(1,2) = cy;
  camera_m.at<double>(2,2) = 1;
  camera_matrix_ = camera_m;
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
