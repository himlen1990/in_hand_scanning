#ifndef GRASPING_DETECTOR_H
#define GRASPING_DETECTOR_H


class grasping_detector
{
 private:
  cv::Mat rgb_;
  //std::vector<hand_keypoint>  keypoints_;
 public:
 grasping_detector()
    {}
 int detect(cv::Mat rgb, std::vector<hand_keypoint> kp)
  {
    int valid_kp_counter = 0;
    rgb.copyTo(rgb_);
#if 1
    if(kp.size() == 4 )
      {
	for(int i =0; i<4; i++)
	  {
	    if(kp[i].get_score() > 0.3)
	      valid_kp_counter++;
	  }
	if (valid_kp_counter>3)
	  {
	    Eigen::Vector2f v1(kp[0].get_uv().x-kp[1].get_uv().x , kp[0].get_uv().y-kp[1].get_uv().y);
	    Eigen::Vector2f v1_norm = v1.normalized();
	
	    
	    Eigen::Vector2f v2(kp[2].get_uv().x-kp[3].get_uv().x , kp[2].get_uv().y-kp[3].get_uv().y);
	    Eigen::Vector2f v2_norm = v2.normalized();
	
	    float finger_theta = acos(v1_norm.dot(v2_norm));
	    
	    float angle = finger_theta * 57.2958;
	    
	    cout<<"angle "<<angle<<endl;
	    if (angle > 130 || angle < 50)
	      return 1;
	    
	//cout<<"touching"<<endl;
	  //cv::line(rgb_,kp[0].get_uv(),kp[1].get_uv(), cv::Scalar(0,255,255));
	  //cv::line(rgb_,kp[2].get_uv(),kp[3].get_uv(), cv::Scalar(0,255,255));

	//cv::imshow("touch_debug",rgb_);
	    else
	      return 0;
	  }
      }
#endif
    return -1;
  }
  
};


#endif
