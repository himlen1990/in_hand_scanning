#include "object_pose_tracker.h"
#include <iostream>


//object edge_detection using threshold in depth

using namespace cv;
using namespace std;
object_pose_tracker::object_pose_tracker(const cv::Mat RGBImg ,const cv::Mat depth, const float lowerb, const float upperb, const boost::array<double,9> camera_info_k):
  rgbImg_(RGBImg), depth_(depth), lowerb_(lowerb),upperb_(upperb), camera_info_k_(camera_info_k)
{

}

bool object_pose_tracker::init_pose(std::vector<cv::Point2f> finger_positions )
{

  if(finger_positions.size()!=2)
    return false;
  
  if(depth_.empty() ||depth_.type()!=5 )
    {
      std::cout<<"depth image is empty or depth image type is not float "<<std::endl;
      return false;
    }
  else
    {
      cv::Mat bin,edge;
      cv::inRange(depth_,cv::Scalar(lowerb_),cv::Scalar(upperb_),bin);
      cv::imshow("bin",bin);

      float fx = camera_info_k_[0];
      float fy = camera_info_k_[4];
      float cx = camera_info_k_[2];
      float cy = camera_info_k_[5];

      int x0 = finger_positions[0].x;
      int x1 = finger_positions[1].x;
      int y0 = finger_positions[0].y;
      int y1 = finger_positions[1].y;

      float midx = (x1 + x0)/2;
      float midy = (y1 + y0)/2;
      float vx = x1 - midx;
      float vy = y1 - midy;
      float mag = sqrt(vx*vx + vy*vy);
      vx = vx/mag;
      vy = vy/mag;
      float temp = vx;
      vx = -vy;
      vy =temp;
      int px = midx-vx*20;
      int py = midy-vy*20;
      cv::circle(rgbImg_,cv::Point(int(midx),int(midy)), 5, cv::Scalar(0,0,255),-1);
      cv::circle(rgbImg_,cv::Point(int(x0),int(y0)), 5, cv::Scalar(255,10,100),-1);
      //cv::circle(rgbImg_,cv::Point(int(fakex),int(fakey)), 5, cv::Scalar(255,0,255),-1);
      cv::circle(rgbImg_,cv::Point(int(x1),int(y1)), 5, cv::Scalar(0,255,255),-1);
      cv::line(rgbImg_,cv::Point(int(midx),int(midy)),cv::Point(int(x1),int(y1)), cv::Scalar(0,255,255));
      cv::line(rgbImg_,cv::Point(int(midx),int(midy)),cv::Point(px,py), cv::Scalar(0,255,255));

      float z0 = depth_.at<float>(y1,x1);
      float z1 = depth_.at<float>(y0,x0);

      float midz = (z0 + z1)/2; //z0 + 0.02;// diff = midz - depth_at.<float>(midx,midy)
      float pz = midz;//depth_.at<float>(py,px) + diff; 
      cout<<"x0 "<<x0 <<" y0 "<<y0<<" z0 "<< depth_.at<float>(y0,x0)<<endl;
      cout<<"x1 "<<x1 <<" y1 "<<y1<<" z1 "<< depth_.at<float>(y1,x1)<<endl;
      //cout<<"x1 "<<midx <<" y1 "<<midy<<" z1 "<< midz <<endl;
      //cout<<"x2 "<<px <<" y2 "<<py<<" z2 "<< pz <<endl;
      std::vector<cv::Point3f> pose3D; //move pose3D later
      imshow( "Coordinate", rgbImg_ ); 

      if(!isnan(z0)  && !isnan(pz) )      
	{
	  cv::Point3f p0((x0 - cx) * z0 / fx, (y0 - cy) * z0 / fy, z0);
	  pose3D.push_back(p0);
	  cv::Point3f p1((midx - cx) * midz /fx, (midy - cy) * midz /fx, midz);
	  pose3D.push_back(p1);
	  cv::Point3f p2((px - cx) * pz /fx, (py - cy) * pz /fx, pz);
	  pose3D.push_back(p2);
	}
      else 
	return false;
      
      /// Show in a window
      Eigen::Vector3f vx3f(pose3D[0].x-pose3D[1].x,pose3D[0].y-pose3D[1].y,pose3D[0].z-pose3D[1].z);
      Eigen::Vector3f vy3f(pose3D[2].x-pose3D[1].x,pose3D[2].y-pose3D[1].y,pose3D[2].z-pose3D[1].z);
      vx3f.normalize();
      vy3f.normalize();
      Eigen::Vector3f vz3f = vx3f.cross(vy3f);
      
      cout<<"!!!!!!!!!!"<<endl;
      /*
      Eigen::Matrix3f rotM = Eigen::Matrix3f::Zero();
      rotM<<vx3f(0),vx3f(1),vx3f(2),
	vy3f(0),vy3f(1),vy3f(2),
	vz3f(0),vz3f(1),vz3f(2);
      */
      Eigen::Matrix4f rotM = Eigen::Matrix4f::Zero();
      rotM<<vx3f(0),vx3f(1),vx3f(2),pose3D[1].x,
	vy3f(0),vy3f(1),vy3f(2),pose3D[1].y,
	vz3f(0),vz3f(1),vz3f(2),pose3D[1].z,
	0, 0, 0, 1;

      rotM_ = rotM;
      orig_ = cv::Point3f(pose3D[1].x,pose3D[1].y,pose3D[1].z);
      cout<<orig_<<endl;
    }
  return true;

}

Eigen::Matrix4f object_pose_tracker::get_3D_pose()
{
  return rotM_;
}

cv::Point3f object_pose_tracker::get_t()
{
  return orig_;
}
