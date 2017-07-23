#include "object_edge_detector.h"
#include <iostream>
//object edge_detection using threshold in depth

using namespace cv;
object_edge_detector::object_edge_detector(const cv::Mat depth, const float lowerb, const float upperb):
  depth_(depth), lowerb_(lowerb),upperb_(upperb)
{}

bool object_edge_detector::detect_edge()
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
      cv::imshow("bin",bin);


      ///contours
      RNG rng(12345);
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      cv::findContours( bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      int largest_area=0;
      int largest_contour_index=0;
      vector<Point>  object_contours;
      for( size_t i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
	  double area = contourArea( contours[i] );  //  Find the area of contour

	  if( area > largest_area )
	    {
	      largest_area = area;
	      largest_contour_index = i;               //Store the index of largest contour
	    }
	}
      cv::Mat drawing = Mat::zeros(bin.size(), CV_8UC3 );
      std::cout<<contours[largest_contour_index].size()<<std::endl;
      for(int i=0; i<contours[largest_contour_index].size(); i++)
	{
	  if(contours[largest_contour_index][i].y<200)
	    {
	      object_contours.push_back(contours[largest_contour_index][i]);
	    }

	}
      
      std::cout<<"after "<<object_contours.size()<<std::endl;
      for(int i=0; i<object_contours.size(); i++)
	{
	  drawing.at<Vec3b>(object_contours[i].y,object_contours[i].x) = cv::Vec3b(255,0,0);
	}
      /// Show in a window
      imshow( "Contours", drawing );
    }
}
