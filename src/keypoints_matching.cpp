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

static const std::string OPENCV_WINDOW1 = "Matching (drawing)";
static const std::string OPENCV_WINDOW2 = "Matching";

std::string DATA_LOCATION_PREFIX = DATA_DIR;


void filterMatches(const std::vector<std::vector<cv::DMatch> > &knn_matches, std::vector<cv::DMatch> &matches) {
  matches.clear();

  for(size_t i = 0; i < knn_matches.size(); i++) {
    if(knn_matches[i].size() >= 2) {
      float ratio = knn_matches[i][0].distance / knn_matches[i][1].distance;

      if(ratio < 0.7) {
        matches.push_back(cv::DMatch(knn_matches[i][0].queryIdx, knn_matches[i][0].trainIdx, knn_matches[i][0].distance));
      }
    }
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "challenge3");

  ros::NodeHandle n(std::string("~"));
  std::string rosbagFileName;
  n.param<std::string>("rosbag_file_name", rosbagFileName, /*DATA_LOCATION_PREFIX + "test.bag"*/ "/media/user/Nouveau nom/Data/Dataset/Udacity/self-driving-car/datasets/data/dataset.bag");

  ros::start();

  ROS_INFO_STREAM("Staring node challenge3!");

  cv::namedWindow(OPENCV_WINDOW1);
  cv::namedWindow(OPENCV_WINDOW2);
  cv::moveWindow(OPENCV_WINDOW2, 0, 500);

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
  
  std::vector<cv::KeyPoint> keypoints_prev, keypoints_curr;
  cv::Mat descriptors_prev, descriptors_curr;
  std::vector<std::vector<cv::DMatch> > knn_matches;
  std::vector<cv::DMatch> matches;
  cv::Mat img_prev, img_cur;
  cv::Mat img_matches, img_matches_draw;

  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
#ifdef USE_OPENCV_3
  cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create();
#else
  // Detect the keypoints using SURF Detector
  int minHessian = 400;

  cv::SurfFeatureDetector detector( minHessian );
#endif
  
  bool quit = false;
  foreach(rosbag::MessageInstance const m, view) {
    
    sensor_msgs::Image::ConstPtr img_msg= m.instantiate<sensor_msgs::Image>();
    if (img_msg != NULL) {
      std::cout << "Image - Timestamp: " << img_msg->header.stamp.toNSec() << " - width: " << img_msg->width << " - height: " << img_msg->height <<  std::endl;

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
      
      cv_ptr->image.copyTo(img_cur);

#ifdef USE_OPENCV_3
      //Detect current keypoints
      detector->detectAndCompute(img_cur, cv::noArray(), keypoints_curr, descriptors_curr);
#else
      // TODO: check this
      detector.detect(img_cur, keypoints_curr);
      detector.compute(img_cur, keypoints_curr, descriptors_curr);
#endif

      if (keypoints_prev.empty()) {
        img_cur.copyTo(img_prev);
        keypoints_prev = keypoints_curr;
        descriptors_curr.copyTo(descriptors_prev);
      }

      knn_matches.clear();
      matcher->knnMatch(descriptors_curr, descriptors_prev, knn_matches, 2);
      filterMatches(knn_matches, matches);

      cv::drawMatches(img_cur, keypoints_curr, img_prev, keypoints_prev, matches, img_matches_draw);
      cv::hconcat(img_cur, img_prev, img_matches);

      float ratio_keypoints_matched = matches.size() / (float) keypoints_curr.size();
      if (ratio_keypoints_matched < 0.5) {
        //Select the current image as the previous image if the matched ratio is < 0.5 (images are too different)
        img_cur.copyTo(img_prev);
        keypoints_prev = keypoints_curr;
        descriptors_curr.copyTo(descriptors_prev);
      }

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW1, img_matches_draw);
      cv::imshow(OPENCV_WINDOW2, img_matches);

      char c = cv::waitKey(10) & 0xFF;
      if (c == 27) {
        quit = true;
      }
    }

    if (quit) {
      break;
    }
  }

  bag.close();

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();

  cv::destroyWindow(OPENCV_WINDOW1);
  cv::destroyWindow(OPENCV_WINDOW2);

  return 0;
}
