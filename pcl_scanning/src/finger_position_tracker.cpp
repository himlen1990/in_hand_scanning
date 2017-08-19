#include "finger_position_tracker.h"
#include <iostream>
//get finger color in init func, then detect that color or directly use kcf
//1.give a bounding box of two fingers, after confirm(press a key), start tracking

finger_position_tracker::finger_position_tracker():init_flag(false),trackers("KCF")
{}

void finger_position_tracker::start(cv::Mat frame, cv::Point thumb, cv::Point forefinger, int bbox_width, int bbox_height)
{
 
      thumb_ = thumb;
      forefinger_ = forefinger;
      thumb_bbox_.x = thumb_.x - bbox_width/2;
      thumb_bbox_.y = thumb_.y - bbox_height/2;
      thumb_bbox_.width = bbox_width;
      thumb_bbox_.height = bbox_height;
      forefinger_bbox_.x = forefinger.x - bbox_width/2;
      forefinger_bbox_.y = forefinger.y - bbox_height/2;
      forefinger_bbox_.width = bbox_width;
      forefinger_bbox_.height = bbox_height;
      init_flag = true;
      //thumb_tracker = cv::Tracker::create( "KCF" );
      //forefinger_tracker = cv::Tracker::create( "KCF" );
      trackers.add(frame, thumb_bbox_);
      trackers.add(frame, forefinger_bbox_);

      rectangle( frame, thumb_bbox_, cv::Scalar( 255, 0, 0 ), 2, 1 );
      rectangle( frame, forefinger_bbox_, cv::Scalar( 255, 0, 0 ), 2, 1 );
      imshow("debug",frame);

}

std::vector<cv::Point> finger_position_tracker::update(cv::Mat frame)
{
  std::vector<cv::Point> finger_positions;

       //thumb_tracker.update(frame,thumb_bbox_);
      //forefinger_tracker.update(frame, forefinger_bbox_);
  // imshow for debug
 
 
      trackers.update(frame);
      for(unsigned i=0;i<trackers.objects.size();i++)
	{
	rectangle( frame, trackers.objects[i], cv::Scalar( 255, 0, 0 ), 2, 1 );
	cv::Point finger_p(trackers.objects[i].x+trackers.objects[i].width/2, trackers.objects[i].y+trackers.objects[i].height/2);
	  finger_positions.push_back(finger_p);
	}
      imshow("debug",frame);
 
  return finger_positions;
}

std::vector<cv::Point> finger_position_tracker::update_using_new_detect_result(cv::Mat frame,cv::Point thumb, cv::Point forefinger)
{
}

