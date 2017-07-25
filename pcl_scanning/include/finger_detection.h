#ifndef FINGER_DETECTION_H
#define FINGER_DETECTION_H
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class finger_detection{

  int iLowH;
  int iHighH;

  int iLowS;
  int iHighS;

  int iLowV;
  int iHighV;

 public:
  finger_detection();
  std::vector<cv::Point> detect(cv::Mat rgbImg);

};
#endif
