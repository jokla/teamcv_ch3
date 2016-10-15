// Include the ROS C++ APIs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

// Include OpenCV
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#if (CV_MAJOR_VERSION >= 3)
#define USE_OPENCV_3
#endif

#ifdef USE_OPENCV_3
#  include <opencv2/xfeatures2d.hpp>
#else
#  include <opencv2/nonfree/features2d.hpp>
#  include <opencv2/nonfree/nonfree.hpp>
#endif



#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv) {

  ros::init(argc, argv, "challenge3");

  ros::NodeHandle n(std::string("~"));
  std::string rosbagFileName;
  n.param<std::string>("rosbag_file_name", rosbagFileName, "test.bag");

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
  std::vector <sensor_msgs::NavSatFix::ConstPtr> gps_vector;
  std::vector <std::vector<cv::KeyPoint> > keypoints_vector;
  unsigned int count = 0;
  bool gps_info_bool = 0;

  foreach(rosbag::MessageInstance const m, view)
  {

    sensor_msgs::NavSatFix::ConstPtr gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
    if (gps_msg != NULL)
    {
      std::cout << "GPS   - Timestamp " << gps_msg->header.stamp.toNSec() << " - Longitude: " << gps_msg->longitude << " Latitude:" <<  gps_msg->latitude << std::endl;
      gps_vector.push_back(gps_msg);
      gps_info_bool = 1;
    }
    sensor_msgs::Image::ConstPtr img_msg= m.instantiate<sensor_msgs::Image>();
    if (img_msg != NULL && gps_info_bool)
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

      std::vector<cv::KeyPoint> keypoints_1;
#ifdef USE_OPENCV_3
      cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create();
      detector->detect( cv_ptr->image, keypoints_1 );
#else
      // Detect the keypoints using SURF Detector
       int minHessian = 400;

       cv::SurfFeatureDetector detector( minHessian );
       detector.detect( cv_ptr->image, keypoints_1 );
       keypoints_vector.push_back(keypoints_1);
#endif

       //-- Draw keypoints
       cv::Mat img_keypoints_1;
       cv::drawKeypoints( cv_ptr->image, keypoints_1, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, img_keypoints_1);
      cv::waitKey(0);
      gps_info_bool = 0;
    }
    count++;
    std::cout << "Skip image " << count << std::endl;

  }

  bag.close();

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  cv::destroyWindow(OPENCV_WINDOW);

  return 0;
}
