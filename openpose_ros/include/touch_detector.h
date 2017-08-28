#ifndef TOUCH_DETECTOR_H
#define TOUCH_DETECTOR_H


class touch_detector
{
 private:
  cv::Mat rgb_;
  //std::vector<hand_keypoint>  keypoints_;
 public:
 touch_detector()
    {}
 bool detect(cv::Mat rgb, std::vector<hand_keypoint> kp)
  {
    //1. check finger angle
    //2. check whether there is a object between two fingers
    rgb.copyTo(rgb_);
    if(kp.size() == 4)
      {
	Eigen::Vector2f v1(kp[0].get_uv().x-kp[1].get_uv().x , kp[0].get_uv().y-kp[1].get_uv().y);
	Eigen::Vector2f v1_norm = v1.normalized();
	float temp = v1_norm(0);
	v1_norm(0) = -v1_norm(1);
	v1_norm(1) = temp;
	int perpend_x1 = kp[1].get_uv().x - v1_norm(0)*20;
	int perpend_y1 = kp[1].get_uv().y - v1_norm(1)*20;
	Eigen::Vector2f force_cone_v1(kp[1].get_uv().x - perpend_x1, kp[1].get_uv().y - perpend_y1);

	//compute object surface normal first, then get the angle!!!
	
	float test_angle = atan(force_cone_v1(1)/force_cone_v1(0))*57.3;



	Eigen::Vector2f v2(kp[2].get_uv().x-kp[3].get_uv().x , kp[2].get_uv().y-kp[3].get_uv().y);
	Eigen::Vector2f v2_norm = v2.normalized();
        temp = v2_norm(0);
	v2_norm(0) = v2_norm(1);
	v2_norm(1) = -temp;
	int perpend_x2 = kp[3].get_uv().x - v2_norm(0)*20;
	int perpend_y2 = kp[3].get_uv().y - v2_norm(1)*20;
	Eigen::Vector2f force_cone_v2(kp[3].get_uv().x - perpend_x2, kp[3].get_uv().y - perpend_y2);

	float test_angle2 = atan(force_cone_v2(1)/force_cone_v2(0))*57.3;
	cout<<"ang1 "<<test_angle<<" ang2 "<<test_angle2<<endl;
	
	if (abs(test_angle) < 35 && abs(test_angle2) < 35)
	  cout<<"force closure"<<endl;
	
	else
	  cout<<"untouching"<<endl;
	
	cv::line(rgb_,kp[0].get_uv(),kp[1].get_uv(), cv::Scalar(0,255,255));
	cv::line(rgb_,cv::Point(perpend_x1,perpend_y1),kp[1].get_uv(), cv::Scalar(0,255,255));

	cv::line(rgb_,kp[2].get_uv(),kp[3].get_uv(), cv::Scalar(0,255,255));
	cv::line(rgb_,cv::Point(perpend_x2,perpend_y2),kp[3].get_uv(), cv::Scalar(0,255,255));

	

	cv::imshow("touch_debug",rgb_);
      }
    return false;
  }
  
};


#endif
