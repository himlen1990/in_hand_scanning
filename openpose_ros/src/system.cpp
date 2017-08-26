#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "hand_keypoint_detector.h"

#include <open_in_hand_scanning/keypoint.h>
using namespace std;
using namespace cv;

class System{
public:

  ros::NodeHandle nh_;
 

  boost::mutex mutex_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_; 
  message_filters::Subscriber<sensor_msgs::Image> image1_sub;
  message_filters::Subscriber<sensor_msgs::Image> image2_sub;
 
  ros::Subscriber camera_info_sub;
  boost::array<double,9> camera_info_k;

  ros::Publisher object_pose_pub;
  geometry_msgs::PoseStamped center;

  ros::Subscriber  hand_keypoint_sub;  

  Eigen::Matrix4f init_pose;
  Eigen::Matrix3f current_pose;
  cv::Point3f orig;

  cv::Mat first_frame;

  vector<float> keypoint_x_;
  vector<float> keypoint_y_;
  
  //-----------------------------
  std::vector<cv::Point2f> finger_position_first;  
  //----------------------------

  System():camera_info_k{0}
  {
    image1_sub.subscribe(nh_, "/camera/rgb/image_rect_color", 1);
    image2_sub.subscribe(nh_, "/camera/depth_registered/image_raw", 1);
    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image > >(10);
    sync_input_2_->connectInput(image1_sub,image2_sub);
    sync_input_2_->registerCallback(boost::bind(&System::callback, this, _1, _2));

    camera_info_sub = nh_.subscribe("/camera/depth_registered/camera_info",1, &System::camera_info_cb,this);

    hand_keypoint_sub = nh_.subscribe("/hand_keypoints",1, &System::hand_keypoint_cb,this);
    
    object_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("object_pose", 100);
  }

  void hand_keypoint_cb(const open_in_hand_scanning::keypoint kp)
  {
    keypoint_x_= kp.keypoint_x;
    keypoint_y_= kp.keypoint_y;

  }


  
  void camera_info_cb(const sensor_msgs::CameraInfoPtr& camInfo)
  {
    camera_info_k = camInfo->K;
  }


    void callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
  {
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImage cv_img;
    try
      {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
      {
        cv_ptrD = cv_bridge::toCvShare(msgD);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    cv::Mat depth;
    cv_ptrD->image.copyTo(depth);
    cv::Mat depth_f;

    cv::Mat RGBImg;
    cv_ptrRGB->image.copyTo(RGBImg);
    cv::cvtColor(RGBImg,RGBImg,CV_BGR2RGB);

    imshow("RGB",RGBImg);
    waitKey(10);

    if (depth.type()==2)
      depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
    else if (depth.type()==5)
      depth_f = depth;
    else
      {
        cout<<"unknown depth Mat type"<<endl;
        return;
      }
    if(keypoint_x_.size()>0)
      if (keypoint_x_[0] >0 )
      {
	for(int i=0; i<keypoint_x_.size(); i++)
	  {
	    cv::circle(RGBImg, cv::Point(int(keypoint_x_[i]),int(keypoint_y_[i])),5, cv::Scalar(255,100,255), 3);
	  }
	imshow("RGB",RGBImg);
	waitKey(10);
      }
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_edge");
  System sys;
  ros::spin();

  return 0;
}
