/*
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //pcl::PCLPointCloud2 pcl_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  //pcl_conversions::moveToPCL(*(const_cast<PointCloud2*>(cloud.get())), pcl_cloud);
  pcl::fromROSMsg (*cloud, pcl_cloud);
  output = *cloud;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_segementation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PCLPointCloud2 PointCloud2;
#else
typedef sensor_msgs::PointCloud2 PointCloud2;
#endif

typedef pcl::PointXYZ Point;
typedef pcl::visualization::PointCloudColorHandler<PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef PointCloud2::Ptr PointCloudPtr;
typedef PointCloud2::ConstPtr PointCloudConstPtr;
PointCloudConstPtr cloud_, cloud_old_;
boost::mutex m;

bool paused = false;
bool record_continuously = false;
bool record_single = false;
std::string topic_name = "";

bool record_fixed_number = false;
int rec_max_frames = 100;
int rec_nr_frames = 0;

void cloud_cb (const PointCloudConstPtr& cloud)
{
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
  std_msgs::Header header = pcl_conversions::fromPCL(cloud->header);
#else
  std_msgs::Header header = cloud->header;
#endif
  float stamp = header.stamp.toSec ();

  ROS_INFO ("PointCloud with %d data points (%s), stamp %f, and frame %s.",
            cloud->width * cloud->height,
            pcl::getFieldsList (*cloud).c_str (),
            stamp,
            cloud->header.frame_id.c_str ());
  m.lock ();

  cloud_ = cloud;

  if(record_continuously || record_single)
    {
      boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%Y_%m_%d_%H_%M_%s");
      std::basic_stringstream<char> ss;
      ss.imbue(std::locale(std::cout.getloc(), facet));
      ss << header.stamp.toBoost();
      std::string formatted_stamp = ss.str();
      replace(formatted_stamp.begin(), formatted_stamp.end(), '.', '_');

      std::string filename = str(boost::format("%s_%s.pcd")
                               % topic_name
				 % formatted_stamp);
      replace(filename.begin(), filename.end(), '/', '_');

    try
      {
	pcl::io::savePCDFile(filename.c_str(),
			     *cloud,
			     Eigen::Vector4f::Zero(),
			     Eigen::Quaternionf::Identity(),
			     true);
	if (record_single)
	  pcl::console::print_info("Stored file %s.\n", filename.c_str());
	record_single = false;
      }
    catch (pcl::IOException& e)
      {
	std::cerr << e.what() << std::endl;
	return;
      }

    if (record_fixed_number)
      if (++rec_nr_frames >= rec_max_frames)
        record_continuously = false;
    }

  m.unlock ();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.keyDown())
    {
      switch (event.getKeyCode())
	{
	case ' ':    // space: grab a single frame
	  record_single = true;
	  break;

	case 'p':   // paused
	  paused = !paused;
	  break;

	case 'K':
	  record_fixed_number = false;
	  record_continuously = !record_continuously;
	  if (record_continuously)
	    std::cerr << "STARTED recording." << std::endl;
	  else
	    std::cerr << "STOPPED recording." << std::endl;
	  break;

	case 'L':
	  record_fixed_number = true;
	  record_continuously = !record_continuously;
	  if (record_continuously)
	    {
	      std::cerr << "STARTED recording." << std::endl;
	      rec_nr_frames = 0;
	    }
	  else
	    std::cerr << "STOPPED recording." << std::endl;
	  break;
	}
    }
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_online_viewer", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  int queue_size = 1;
  pcl::console::parse_argument (argc, argv, "-qsize", queue_size);

  bool headless = false;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("headless", headless);
  if (headless)
    {
      ROS_WARN("Running in headless mode. All point clouds are written to disk");
      record_continuously = true; // no viewer window, all point clouds are written to disk
    }

  ros::Subscriber sub = nh.subscribe ("input", queue_size, cloud_cb);
  topic_name = ros::names::remap("input").c_str();
  pcl::console::print_highlight("Subscribing to %s using a queue size of %d\n", topic_name.c_str(), queue_size);

  pcl::visualization::PCLVisualizer::Ptr p;
  ColorHandlerPtr color_handler;
  pcl::PointCloud<Point>::Ptr cloud_xyz;

  if (!headless)
    {
      p.reset(new pcl::visualization::PCLVisualizer(argc, argv, "Online PointCloud2 Viewer"));
      cloud_xyz.reset(new pcl::PointCloud<Point>);
      p->registerKeyboardCallback(keyboardEventOccurred, (void*)&p);
    }

  int color_handler_idx = 0;
  double psize = 0;
  while (nh.ok ())
    {
      ros::spinOnce ();
      ros::Duration (0.001).sleep ();

      if (headless)
	{
	  ros::Duration (0.001).sleep ();
	}
      else
	{
	  p->spinOnce (10);

	  if (!cloud_)
	    continue;

	  if (cloud_ == cloud_old_)
	    continue;

	  color_handler_idx = p->getColorHandlerIndex ("cloud");
	  p->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");
	  p->removePointCloud ("cloud");
	  m.lock ();
	  {
	    // filter out NaNs
	    pcl::PassThrough<PointCloud2> filter;
	    PointCloud2::Ptr cloud_filtered(new PointCloud2);
	    filter.setInputCloud(cloud_);
	    filter.filter(*cloud_filtered);

	    // convert point cloud to PCL PointCloud type
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
	    pcl::fromPCLPointCloud2 (*cloud_filtered, *cloud_xyz);
#else
	    pcl::fromROSMsg (*cloud_, *cloud_xyz);
#endif

	    // create color handlers
	    color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<PointCloud2> (cloud_, 255.0, 1.0, 1.0));
	    p->addPointCloud<Point>(cloud_xyz, color_handler, "cloud");
	    for (size_t i = 0; i < cloud_->fields.size (); ++i)
	      {
		if (cloud_->fields[i].name == "rgb" || cloud_->fields[i].name == "rgba")
		  color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud2> (cloud_));
		else
		  color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<PointCloud2> (cloud_, cloud_->fields[i].name));
		p->addPointCloud<Point>(cloud_xyz, color_handler, "cloud");
	      }
	    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");
	    if (color_handler_idx != -1)
	      p->updateColorHandlerIndex ("cloud", color_handler_idx);
	    cloud_old_ = cloud_;
	  }
	  m.unlock ();
	}

    }
  return (0);
}
