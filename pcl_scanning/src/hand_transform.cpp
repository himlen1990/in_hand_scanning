#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
//#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sstream>

using namespace std;

class pointProject{
public:
  ros::NodeHandle nh_;
  boost::mutex mutex_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, geometry_msgs::PoseArray> >* sync_input_2_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub;
  message_filters::Subscriber<geometry_msgs::PoseArray> point2d_sub;
//  ros::Publisher trackerbox_center_pub;
//  ros::Publisher target_lost_pub;
//  tf::TransformListener listener;
//  tf::StampedTransform transform;
//  geometry_msgs::PointStamped center;
//  geometry_msgs::PointStamped pt_transformed;
//  double timer_start,timer_now;
//  float pre_x, pre_y, pre_z;
  bool init_flag;
  
  pointProject()
  {
//    trackerbox_center_pub = nh_.advertise<geometry_msgs::PointStamped>("trackerbox_center_3D", 1);
//  target_lost_pub = nh_.advertise<std_msgs::String>("follow_me/target_lost", 1);
    points_sub.subscribe(nh_,"/hsrb/head_rgbd_sensor/depth_registered/rectified_points",1);
    point2d_sub.subscribe(nh_,"hand_keypoints",1);
    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, geometry_msgs::PoseArray> >(1);
    sync_input_2_->connectInput(points_sub,point2d_sub);
    sync_input_2_->registerCallback(boost::bind(&pointProject::positionCb, this, _1, _2));
    //    init_flag = false;
    //    pre_x = 0;
    //    pre_y = 0;
    //    pre_z = 0;
  }
  ~pointProject(){}

  void positionCb ( const sensor_msgs::PointCloud2::ConstPtr& points, const geometry_msgs::PoseArray::ConstPtr& hand_keypoints)
  {

    pcl::PointCloud < pcl::PointXYZRGB > depth_cloud;
    pcl::fromROSMsg(*points, depth_cloud);
    int x0 =hand_keypoints->poses[0].position.x;
    int y0 =hand_keypoints->poses[0].position.y;    
    int x1 =hand_keypoints->poses[1].position.x;
    int y1 =hand_keypoints->poses[1].position.y;    
    int x2 =hand_keypoints->poses[2].position.x;
    int y2 =hand_keypoints->poses[2].position.y;    

    pcl::PointXYZRGB p0,p1,p2;
    p0 = depth_cloud.points[depth_cloud.width * y0 + x0] ;
    p1 = depth_cloud.points[depth_cloud.width * y1 + x1] ;
    p2 = depth_cloud.points[depth_cloud.width * y2 + x2] ;
  
    if ( !isnan (p0.x) && ((p0.x != 0.0) || (p0.y != 0.0) || (p0.z == 0.0)) &&
	 !isnan (p1.x) && ((p1.x != 0.0) || (p1.y != 0.0) || (p1.z == 0.0)) &&
	 !isnan (p2.x) && ((p2.x != 0.0) || (p2.y != 0.0) || (p2.z == 0.0)) )
      {
	cout<<"!!!"<<endl;
      }
  }
};
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "hand_transform");
    pointProject pp;
    ros::spin();
    return 0;
  }

