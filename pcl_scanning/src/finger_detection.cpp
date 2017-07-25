#include "finger_detection.h"

using namespace cv;
using namespace std;
finger_detection::finger_detection():iLowH(100),iHighH(120),iLowS(150),iHighS(250),iLowV(120),iHighV(255) {}


std::vector<cv::Point> finger_detection::detect(cv::Mat rgbImg)
  {
    std::vector<cv::Point> finger_positions;
    cv::Mat HSVImg,BinImg;
    cv::cvtColor(rgbImg,HSVImg,CV_RGB2HSV);

    cv::inRange(HSVImg, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), BinImg);

    int largest_area=0;
    int largest_contour_index=0;
    int largest_area_2=0;
    int largest_contour_index_2=0;
    vector<vector<Point> > contours; 
    findContours(BinImg, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    for( size_t i = 0; i< contours.size(); i++ ) 
      {
        double area = contourArea( contours[i] );  

        if( area > largest_area )
	  {
	    largest_area_2 = largest_area;
	    largest_contour_index_2 = largest_contour_index;
            largest_area = area;
            largest_contour_index = i;              
	  }
	if( area > largest_area_2 && area < largest_area)
	  {
            largest_area_2 = area;
            largest_contour_index_2 = i;              
	  }
      }

    if(largest_contour_index != largest_contour_index_2 && largest_area> 300 && largest_area_2 >300)
      {
	drawContours( rgbImg, contours,largest_contour_index, Scalar( 0, 255, 0 ), 2 );
	drawContours( rgbImg, contours,largest_contour_index_2, Scalar( 0, 255, 0 ), 2 );

	Moments M1,M2;
	Point2f MC1,MC2;
	M1 = moments(contours[largest_contour_index],false);
	M2 = moments(contours[largest_contour_index_2],false);

	MC1 = Point( int(M1.m10/M1.m00) , int(M1.m01/M1.m00) ); 
	MC2 = Point( int(M2.m10/M2.m00) , int(M2.m01/M2.m00) ); 
      
	Scalar color = Scalar(255,255,255);
	circle( rgbImg, MC1, 4, Scalar(255,0,0), -1, 8, 0 );
	circle( rgbImg, MC2, 4, color, -1, 8, 0 );
	if (MC1.x < MC2.x)
	  {
	finger_positions.push_back(MC1);
    	finger_positions.push_back(MC2);
	  }
	else
	  {
	finger_positions.push_back(MC2);
    	finger_positions.push_back(MC1);
	  }
	//cout<<DImg.type()<<endl;
      }

    cv::imshow("src",rgbImg);
    cv::waitKey(15);
    return finger_positions;
  }


