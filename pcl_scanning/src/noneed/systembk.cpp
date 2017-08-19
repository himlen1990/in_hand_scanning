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

#include "object_pose_detector.h"
#include "finger_detection.h"
#include "finger_position_tracker.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

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
  finger_position_tracker fpt;
  bool tracker_init;
  boost::array<double,9> camera_info_k;

  ros::Publisher object_pose_pub;
  geometry_msgs::PoseStamped center;


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
    std::vector<cv::Point> finger_position;

    finger_position = fd.detect(RGBImg);
#if 0
    if(finger_position.size()>0 && !tracker_init)
      {
	fpt.start(RGBImg,finger_position[0],finger_position[1],50,50);
	tracker_init = true;
      }
    if(tracker_init)
      {
	finger_position = fpt.update(RGBImg);
      }

    std::vector<cv::Point3f> pose3D;
    if(camera_info_k[0]!=0)
      {
        object_pose_detector opd(RGBImg,depth_f,0.3,0.8,camera_info_k);
        if(opd.detect_pose(finger_position))
          pose3D = opd.get_3D_pose();
	
      }
    //cout<<pose3D[0]<<endl;
    if (pose3D.size()>0)
      {
	Eigen::Vector3f vx(pose3D[2].x-pose3D[1].x,pose3D[2].y-pose3D[1].y,pose3D[2].z-pose3D[1].z);
	Eigen::Vector3f vy(pose3D[0].x-pose3D[1].x,pose3D[0].y-pose3D[1].y,pose3D[0].z-pose3D[1].z);
        vx.normalize();
        vy.normalize();
	Eigen::Vector3f vz = vx.cross(vy);


	Eigen::Matrix3f rotM = Eigen::Matrix3f::Zero();
        rotM<<vx(0),vx(1),vx(2),
          vy(0),vy(1),vy(2),
          vz(0),vz(1),vz(2);
	Eigen::Quaternionf q(rotM);

        center.pose.position.x=pose3D[1].x;
        center.pose.position.y=pose3D[1].y;
        center.pose.position.z=pose3D[1].z;
        center.pose.orientation.x = q.x();
        center.pose.orientation.y = q.y();
        center.pose.orientation.z = q.z();
        center.pose.orientation.w = q.w();
        center.header=msgRGB->header;
	object_pose_pub.publish(center);
      }
#endif    
    cv::waitKey(1);


  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_edge");
  System sys;
  ros::spin();

  return 0;
}
