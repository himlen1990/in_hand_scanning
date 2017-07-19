#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sstream>

using namespace std;

class pointProject{
public:
  ros::NodeHandle nh_;
  boost::mutex mutex_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, geometry_msgs::PointStamped> >* sync_input_2_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub;
  message_filters::Subscriber<geometry_msgs::PointStamped> point2d_sub;
  ros::Publisher trackerbox_center_pub;
  ros::Publisher target_lost_pub;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::PointStamped center;
  geometry_msgs::PointStamped pt_transformed;
  double timer_start,timer_now;
  float pre_x, pre_y, pre_z;
  bool init_flag;
  
  pointProject()
  {
    trackerbox_center_pub = nh_.advertise<geometry_msgs::PointStamped>("trackerbox_center_3D", 1);
    target_lost_pub = nh_.advertise<std_msgs::String>("follow_me/target_lost", 1);
    points_sub.subscribe(nh_,"/hsrb/head_rgbd_sensor/depth_registered/rectified_points",1);
    point2d_sub.subscribe(nh_,"trackerbox_center",1);
    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, geometry_msgs::PointStamped> >(1);
    sync_input_2_->connectInput(points_sub,point2d_sub);
    sync_input_2_->registerCallback(boost::bind(&pointProject::positionCb, this, _1, _2));
    init_flag = false;
    pre_x = 0;
    pre_y = 0;
    pre_z = 0;
  }
  ~pointProject(){}

  void positionCb ( const sensor_msgs::PointCloud2::ConstPtr& points, const geometry_msgs::PointStamped::ConstPtr& imageposition)
  {

    pcl::PointCloud < pcl::PointXYZRGB > depth_cloud;
    pcl::fromROSMsg(*points, depth_cloud);
    int x = imageposition->point.x;
    int y = imageposition->point.y;    
    pcl::PointXYZRGB p;
    p = depth_cloud.points[depth_cloud.width * y + x] ;
    if ( !isnan (p.x) && ((p.x != 0.0) || (p.y != 0.0) || (p.z == 0.0)) )
      {

	if(!init_flag)
	  {
	    pre_x = p.x;
	    pre_y = p.y;
	    pre_z = p.z;
	    init_flag = true;
            timer_start=ros::Time::now().toSec();
            timer_now=ros::Time::now().toSec();
	    cout<<"init"<<endl;
	  }
	else
	  {
	    if(abs(p.x-pre_x)<0.5 && abs(p.y-pre_y)<0.5 && abs(p.z-pre_z)<0.5 
	       && (abs(p.x-pre_x)>0.05 || abs(p.y-pre_y)>0.05 || abs(p.z-pre_z)>0.05))
	      {
		center.point.x = p.x;
		center.point.y = p.y;
		center.point.z = p.z; 
		pre_x = p.x;
		pre_y = p.y;
		pre_z = p.z;
		center.header.stamp = imageposition->header.stamp;
		center.header.frame_id = "/head_rgbd_sensor_rgb_frame";        
		timer_start=ros::Time::now().toSec();
		try{
        
		  listener.transformPoint("/base_link", center, pt_transformed);
		  trackerbox_center_pub.publish(pt_transformed);
		}
		catch (tf::TransformException ex){
		}
	      }//end of if(abs()
	    else if(abs(p.x-pre_x)<0.05 && abs(p.y-pre_y)<0.05 && abs(p.z-pre_z)<0.05)
	      {
		timer_now=ros::Time::now().toSec();
		if((timer_now-timer_start)>5.0)
		  {
		  cout<<"target not moving!!!!"<<endl;
		  init_flag = false;
		  std_msgs::String target_stop;
		  std::stringstream sss;
		  sss << "stop";
		  target_stop.data = sss.str();
		  target_lost_pub.publish(target_stop);
		  cout<<"stop px py pz"<<pre_x<<" "<<pre_y<<" "<<pre_z<<endl;
		  }
		
	      }
	    else 
	      {
		cout<<"after stop px py pz"<<pre_x<<" "<<pre_y<<" "<<pre_z<<endl;
		cout<<"center point moving to fast! > 0.5m in one frame"<<endl;
		cout<<abs(p.x-pre_x)<<endl;
		cout<<abs(p.y-pre_y)<<endl;
		cout<<abs(p.z-pre_z)<<endl;
		init_flag = false;
		std_msgs::String target_lost;
		std::stringstream ss;
                ss << "lost";
                target_lost.data = ss.str();
                target_lost_pub.publish(target_lost);

	      }
	  }//end of else
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

