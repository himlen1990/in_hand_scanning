#ifndef HAND_KEYPOINT_DETECTOR_H
#define HAND_KEYPOINT_DETECTOR_H

#define USE_CAFFE

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <typeinfo>
#include <opencv2/core/core.hpp>

// C++ std library dependencies                                    
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <gflags/gflags.h>
#include <glog/logging.h> 
#include <openpose/headers.hpp>
#include <string>

#include <open_in_hand_scanning/keypoint.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cam_motion_tracker.h"

class hand_keypoint_detector
{
  public:
  ros::NodeHandle nh_;
  //image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  ros::Publisher hand_keypoints_pub_;
  ros::Subscriber camera_info_sub;
  boost::array<double,9> camera_info;
  bool get_camera_info;
  boost::mutex mutex_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_;
  message_filters::Subscriber<sensor_msgs::Image> image1_sub;
  message_filters::Subscriber<sensor_msgs::Image> image2_sub;
  cv::Mat rgbImg_;
  cv::Mat depth_f_;
  bool kinect;

  cam_motion_tracker cmt;

  bool init_flag;
  
  //------------------------------------
#if 0
  string camera_topic;
  string model_folder;
  string resolution;
  int num_gpu;
  int num_gpu_start;
  int keypoint_scale;
  string model_pose;
  string net_resolution;
  int scale_number;
  double scale_gap;
  bool heatmaps_add_parts;
  bool heatmaps_add_bkg;
  bool heatmaps_add_PAFs;
  int heatmaps_scale;
  bool hand;
  string hand_net_resolution;
  int hand_scale_number;
  double hand_scale_range;
  bool hand_tracking;
  int part_to_show;
  bool disable_blending;
  double render_threshold;
  string write_coco_json;
  string write_heatmaps;
  string write_heatmaps_format;
  //------------------------------------
#endif
 public:
  hand_keypoint_detector(const std::string& image_topic);
  void camera_info_cb(const sensor_msgs::CameraInfoPtr& camInfo);
  void convertImage(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
  std::vector<std::vector<float> > get_keypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);
  std::shared_ptr<std::vector<op::Datum>> createDatum();

};
#endif
