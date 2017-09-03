#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
//#include "hand_keypoint_detector.h"

//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl_ros/point_cloud.h>
#include <open_in_hand_scanning/keypoint.h>
#include <pcl/features/normal_3d.h>
using namespace std;
using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
class System{
public:

  ros::NodeHandle nh_;
 

  boost::mutex mutex_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_3_; 
  message_filters::Subscriber<sensor_msgs::Image> image1_sub;
  message_filters::Subscriber<sensor_msgs::Image> image2_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
 
  ros::Subscriber camera_info_sub;
  boost::array<double,9> camera_info_k;

  ros::Publisher object_pose_pub;
  geometry_msgs::PoseStamped center;

  ros::Subscriber  hand_keypoint_sub;  

  ros::Publisher my_pointcloud_pub;

  Eigen::Matrix4f init_pose;
  Eigen::Matrix3f current_pose;
  cv::Point3f orig;

  cv::Mat first_frame;

  vector<float> keypoint_x_;
  vector<float> keypoint_y_;
  
  //-----------------------------
  std::vector<cv::Point2f> finger_position_first;  
  //----------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy;
  bool init_flag;

  System():camera_info_k{0}, copy(new pcl::PointCloud<pcl::PointXYZRGB>()),init_flag(false)
  {
    image1_sub.subscribe(nh_, "/camera/rgb/image_rect_color", 1);
    image2_sub.subscribe(nh_, "/camera/depth_registered/image_raw", 1);
    pointcloud_sub.subscribe(nh_, "/camera/depth_registered/points", 1);
    sync_input_3_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::PointCloud2 > >(10);
    sync_input_3_->connectInput(image1_sub,image2_sub,pointcloud_sub);
    sync_input_3_->registerCallback(boost::bind(&System::callback, this, _1, _2,_3));

    camera_info_sub = nh_.subscribe("/camera/depth_registered/camera_info",1, &System::camera_info_cb,this);

    hand_keypoint_sub = nh_.subscribe("/hand_keypoints",1, &System::hand_keypoint_cb,this);
    
    object_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("object_pose", 100);
    
    my_pointcloud_pub = nh_.advertise<PCLCloud> ("myoutput", 1);
    
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


  void callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::PointCloud2::ConstPtr& points)
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


    //deal with point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());  
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*points, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *cloud);
#if 0
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 1.0)));

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);

    condrem.setKeepOrganized(true);
    condrem.filter (*cloud_out);
#endif

    if(!init_flag)
      {
	copyPointCloud(*cloud,*copy);
	init_flag = true;
      }

    else
      copyPointCloud(*copy,*cloud_out);
    cout<<copy->points.size()<<endl;
    //copyPointCloud(*cloud,*cloud_out);
    sensor_msgs::PointCloud2 pc2;  
    pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2 (*cloud_out, *pcl_pc_2);
    pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
    my_pointcloud_pub.publish(pc2);

    imshow("RGB",RGBImg);
    waitKey(10);

  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_edge");
  System sys;
  ros::spin();

  return 0;
}
