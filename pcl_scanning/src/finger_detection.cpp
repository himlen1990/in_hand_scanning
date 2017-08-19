#include "finger_detection.h"

using namespace cv;
using namespace std;
finger_detection::finger_detection():iLowH(100),iHighH(122),iLowS(180),iHighS(240),iLowV(135),iHighV(250) {}


std::vector<cv::Point2f> finger_detection::detect(cv::Mat rgbImg)
  {
    std::vector<cv::Point2f> finger_positions;
    cv::Mat HSVImg,BinImg;
    cv::cvtColor(rgbImg,HSVImg,CV_RGB2HSV);

    cv::inRange(HSVImg, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), BinImg);


    vector<vector<Point> > contours; 

    findContours(BinImg, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    vector<pair<double,int> > areaP;

    for( size_t i = 0; i< contours.size(); i++ ) 
      {
        double area = contourArea( contours[i] );  
	areaP.push_back(make_pair(area,i));
      }
    
    sort(areaP.begin(),areaP.end());
    reverse(areaP.begin(),areaP.end());

    if(areaP.size() > 4)
      if(areaP[3].first > 50)
	{

	  vector<Moments> M;
	  vector<pair<int,int> > point;
	  Scalar color = Scalar(255,255,255);
	  for(int i=0; i<4; i++)
	    {
	      drawContours( rgbImg, contours,areaP[i].second, Scalar( 0, 255, 0 ), 2 );
	      Moments m = moments(contours[contours,areaP[i].second],false);	      
	      cv::Point mc = Point( int(m.m10/m.m00) , int(m.m01/m.m00) ); 
	      circle( rgbImg, mc, 4, color, -1, 8, 0 );
	      M.push_back(m);
	      point.push_back(make_pair(mc.x,mc.y));
	    }
	  
	  sort(point.begin(),point.end());
	  
	  if(point[0].second < point[1].second)
	    {
	      finger_positions.push_back(cv::Point2f(point[0].first,point[0].second));
	      finger_positions.push_back(cv::Point2f(point[1].first,point[1].second));
	      
	      if(point[2].second < point[3].second)
		{
		  finger_positions.push_back(cv::Point2f(point[2].first,point[2].second));
		  finger_positions.push_back(cv::Point2f(point[3].first,point[3].second));	     
		}
	      else
		{
		  finger_positions.push_back(cv::Point2f(point[3].first,point[3].second));
		  finger_positions.push_back(cv::Point2f(point[2].first,point[2].second));	     
		}
	    }

	  else
	    {
	      finger_positions.push_back(cv::Point2f(point[1].first,point[1].second));
	      finger_positions.push_back(cv::Point2f(point[0].first,point[0].second));
	      
	      if(point[2].second < point[3].second)
		{
		  finger_positions.push_back(cv::Point2f(point[2].first,point[2].second));
		  finger_positions.push_back(cv::Point2f(point[3].first,point[3].second));	     
		}
	      else
		{
		  finger_positions.push_back(cv::Point2f(point[3].first,point[3].second));
		  finger_positions.push_back(cv::Point2f(point[2].first,point[2].second));	     
		}
	    }
	}

    cv::imshow("src",rgbImg);
    cv::waitKey(15);
    return finger_positions;
  }


