// Include the ROS C++ APIs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

static const std::string OPENCV_WINDOW = "Image window";

std::string DATA_LOCATION_PREFIX = DATA_DIR;


int main(int argc, char** argv) {

  ros::init(argc, argv, "challenge3");

  ros::NodeHandle n(std::string("~"));
  std::string rosbagFileName;
  n.param<std::string>("rosbag_file_name", rosbagFileName, DATA_LOCATION_PREFIX + "test.bag");

  ros::start();

  ROS_INFO_STREAM("Staring node challenge3!");

  cv::namedWindow(OPENCV_WINDOW);

  rosbag::Bag bag;

  try {
    bag.open(rosbagFileName, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR("Cannot open the rosbag");
    return 0;
  }

  std::vector<std::string> topics;
  topics.push_back(std::string("/vehicle/gps/fix"));
  topics.push_back(std::string("/center_camera/image_color"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const m, view)
  {

    sensor_msgs::NavSatFix::ConstPtr gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
    if (gps_msg != NULL)
      std::cout << "GPS   - Timestamp " << gps_msg->header.stamp.toNSec() << " - Longitude: " << gps_msg->longitude << " Latitude:" <<  gps_msg->latitude << std::endl;

    sensor_msgs::Image::ConstPtr img_msg= m.instantiate<sensor_msgs::Image>();
    if (img_msg != NULL)
    {
      std::cout << "Image - Timestamp " << img_msg->header.stamp.toNSec() << " - width: " << img_msg->width <<  std::endl;

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return 0;
      }
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(100);
    }
  }

  bag.close();

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  cv::destroyWindow(OPENCV_WINDOW);

  return 0;
}
