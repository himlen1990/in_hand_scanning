#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include "point_cloud_processor.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
class point_cloud_processor
{


 public:
  point_cloud_processor()
    {}

  void distance_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
    {
      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, -0.17)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, 0.05)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, -0.15)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 0.17)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.3)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 0.76)));

      pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
      condrem.setCondition (range_cond);
      condrem.setInputCloud (cloud_in);
      condrem.setKeepOrganized(true);
      condrem.filter (*cloud_out);
    }
  
};


#endif
