#include "object_pose_detector.h"
#include <iostream>

//object edge_detection using threshold in depth

using namespace cv;
using namespace std;
object_pose_detector::object_pose_detector(const cv::Mat RGBImg ,const cv::Mat depth, const float lowerb, const float upperb):
  rgbImg_(RGBImg), depth_(depth), lowerb_(lowerb),upperb_(upperb)
{

}

bool object_pose_detector::detect_pose(std::vector<cv::Point> finger_positions)
{

  if(depth_.empty() ||depth_.type()!=5)
    {
      std::cout<<"depth image is empty or depth image type is not float"<<std::endl;
      return false;
    }
  else
    {
      cv::Mat bin,edge;
      cv::inRange(depth_,cv::Scalar(lowerb_),cv::Scalar(upperb_),bin);
      //cv::imshow("bin",bin);

      ///contours
      RNG rng(12345);
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      cv::findContours( bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      int largest_area=0;
      int largest_contour_index=0;
      vector<Point>  object_contours;
      for( size_t i = 0; i< contours.size(); i++ ) 
	{
	  double area = contourArea( contours[i] ); 

	  if( area > largest_area )
	    {
	      largest_area = area;
	      largest_contour_index = i;            
	    }
	}


      cv::Mat drawing = Mat::zeros(bin.size(), CV_8UC3 );
      if(contours.size()>0)
	{
	  for(int i=0; i<contours[largest_contour_index].size(); i++)
	    {
	      if(contours[largest_contour_index][i].y< finger_positions[0].y)
		
		object_contours.push_back(contours[largest_contour_index][i]);
	    } 
	}
      

      for(int i=0; i<object_contours.size(); i++)
	{
	  drawing.at<Vec3b>(object_contours[i].y,object_contours[i].x) = cv::Vec3b(255,0,0);
	}

      /// Show in a window
      imshow( "Contours", drawing );

 
    }
  return true;

}
