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

#include "object_pose_tracker.h"
#include "finger_detection.h"
#include "cam_motion_tracker.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;

class System{
public:

  ros::NodeHandle nh_;
 

  boost::mutex mutex_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_; 
  message_filters::Subscriber<sensor_msgs::Image> image1_sub;
  message_filters::Subscriber<sensor_msgs::Image> image2_sub;
 
  ros::Subscriber camera_info_sub;
  finger_detection fd;
  cam_motion_tracker cmt;
  bool tracker_init;
  boost::array<double,9> camera_info_k;

  ros::Publisher object_pose_pub;
  geometry_msgs::PoseStamped center;


  Eigen::Matrix4f init_pose;
  Eigen::Matrix3f current_pose;
  cv::Point3f orig;


  //-----------------------------
  std::vector<cv::Point2f> finger_position_first;  
  //----------------------------

  System():tracker_init(false), camera_info_k{0}
  {
    image1_sub.subscribe(nh_, "/camera/rgb/image_rect_color", 1);
    //image1_sub.subscribe(nh_, "/hsrb/head_rgbd_sensor/rgb/image_rect_color", 1);
    image2_sub.subscribe(nh_, "/camera/depth_registered/image_raw", 1);
    //image2_sub.subscribe(nh_, "/hsrb/head_rgbd_sensor/depth_registered/image_raw", 1);
    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image > >(10);
    sync_input_2_->connectInput(image1_sub,image2_sub);
    sync_input_2_->registerCallback(boost::bind(&System::callback, this, _1, _2));

    //camera_info_sub = nh_.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info",1, &System::camera_info_cb,this);

    camera_info_sub = nh_.subscribe("/camera/depth_registered/camera_info",1, &System::camera_info_cb,this);
    
    object_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("object_pose", 100);
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
    //cv::cvtColor(RGBImg,RGBImg,CV_BGR2RGB);

    if (depth.type()==2)
      depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
    else if (depth.type()==5)
      depth_f = depth;
    else
      {
        cout<<"unknown depth Mat type"<<endl;
        return;
      }


    std::vector<cv::Point2f> finger_position;

    finger_position = fd.detect(RGBImg);

#if 0
    if(finger_position.size()>3 && !tracker_init )
      {
	finger_position_first = finger_position;
	tracker_init = true;
      }

    Mat inlier_mask;
    if(finger_position.size()>3 && tracker_init)
      {
	Mat homography = findHomography(finger_position_first , finger_position,
					RANSAC, 2.5f, inlier_mask);	
	float x = atan2(homography.at<double>(2,1),1.0);
	float y = atan2(-homography.at<double>(2,0), sqrt(homography.at<double>(2,1)*homography.at<double>(2,1) + 1 ));
	float z = atan2(homography.at<double>(1,0),homography.at<double>(0,0));
	cout<<"x "<< x*57.3<<endl;
	cout<<"y "<< y*57.3<<endl;
	cout<<"z "<< z*57.3<<endl;
      }

#endif


#if 1
    if(finger_position.size()>1 && !tracker_init )
      {
	int x = finger_position[0].x-50;
	if (x < 0)
	  x = 0;
	int y = finger_position[0].y-50;
	if (y < 0)
	  y = 0;
	int width = finger_position[1].x -finger_position[0].x+100;
	if (x + width > RGBImg.cols)
	  width = RGBImg.cols - x;
	int height = 150;
	if (y + height > RGBImg.rows)
	  height = RGBImg.rows - y;
	if(camera_info_k[0]!=0)
	  {
	    object_pose_tracker opt(RGBImg,depth_f,0.3,0.8,camera_info_k);

	    cmt.set_camera_info(camera_info_k);

	    if (width>110 && height > 100 && opt.init_pose(finger_position)) 
	      {
		cv::Rect roi(x,y,width,height);
		cmt.start(RGBImg,roi);
		init_pose = opt.get_3D_pose();
		current_pose = init_pose.block<3,3>(0,0);
		
		tracker_init = true;
	      }
	  }
      }
    if(tracker_init)
      {
	Eigen::Matrix4f P_matrix = cmt.update(RGBImg,depth_f);
	//cout<<P_matrix<<endl;
	if((P_matrix.array() != 0.0).any())
	  {
	    
	    current_pose =  current_pose * P_matrix.block<3,3>(0,0);
	    //cout<<current_pose<<endl;

	  }	
      }
#endif
    //std::vector<cv::Point3f> pose3D;
    //object_pose_detector opd(RGBImg,depth_f,0.3,0.8,camera_info_k);

	

    //cout<<pose3D[0]<<endl;
    if ((current_pose.array() != 0.0).any())
      {

	//cout<<tra<<endl;
#if 1
	Eigen::Quaternionf q(current_pose);

	//cout<<orig<<endl;
        center.pose.position.x=init_pose(0,3);
        center.pose.position.y=init_pose(1,3);
        center.pose.position.z=init_pose(2,3);
        center.pose.orientation.x = q.x();
        center.pose.orientation.y = q.y();
        center.pose.orientation.z = q.z();
        center.pose.orientation.w = q.w();
        center.header=msgRGB->header;
	object_pose_pub.publish(center);
#endif
      }

    cv::waitKey(50);


  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_edge");
  System sys;
  ros::spin();

  return 0;
}
