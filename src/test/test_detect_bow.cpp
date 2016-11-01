#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#if (CV_MAJOR_VERSION >= 3)
#include <opencv2/xfeatures2d.hpp>

#define DEBUG 1


const char* keys =
{
  "{ help h       |                | print help message }"
  "{ directory d  |                | images directory }"
  "{ image i      |                | image }"
  "{ detector     | SURF           | detector name }"
  "{ extractor    | SURF           | extractor name }"
  "{ dictionary   | dictionary.bin | dictionary filename }"
  "{ book         | book.bin       | book filename }"
  "{ train_dir    |                | training image directory }"
};

void readBOWDictionary(const std::string &intput_filename, cv::Mat &dictionary) {
  std::ifstream file(intput_filename.c_str(), std::ifstream::binary);

  if (!file.is_open()) {
    std::cerr << "Cannot I/O file: " << intput_filename << std::endl;
    return;
  }

  //Read rows, cols, type
  int rows = 0, cols = 0, type = 0;
  file.read((char *)(&rows), sizeof(rows));
  file.read((char *)(&cols), sizeof(cols));
  file.read((char *)(&type), sizeof(type));

  dictionary = cv::Mat(rows, cols, type);

  for (int i = 0; i < dictionary.rows; i++) {
    for (int j = 0; j < dictionary.cols; j++) {
      float val = 0.0f;
      file.read((char *)(&val), sizeof(val));

      dictionary.ptr<float>(i)[j] = val;
    }
  }
}

void init_feature(const std::string &feature_name, cv::Ptr<cv::Feature2D> &feature) {
  if (feature_name == "SIFT") {
    feature = cv::xfeatures2d::SIFT::create();
  } else if (feature_name == "SURF") {
    feature = cv::xfeatures2d::SURF::create(100, 4, 3, true);
  } else if (feature_name == "FAST") {
    feature = cv::FastFeatureDetector::create();
  } else {
    feature = cv::xfeatures2d::SURF::create(100, 4, 3, true);
  }
}

std::pair<std::string, double> compareHist(const std::string &book_filename, const cv::Mat &bow_descriptors) {
  std::ifstream file(book_filename.c_str(), std::ifstream::binary);
  if (!file.is_open()) {
    std::cerr << "Cannot I/O file: " << book_filename << std::endl;
    return std::pair<std::string, double>();
  }

  //Read size and type
  int nb_images = 0, cluster_size = 0, type = 0;
  file.read((char *)(&nb_images), sizeof(nb_images));
  file.read((char *)(&cluster_size), sizeof(cluster_size));
  file.read((char *)(&type), sizeof(type));

  double best_dist = 0.0;
  std::string best_timestamp = "";
  for (int cpt = 0; cpt < nb_images; cpt++) {
    //Read timestamp
    int length = 0;
    file.read((char *)(&length), sizeof(length));

    char* timestamp_char = new char[length + 1];
    for(int idx = 0; idx < length; idx++) {
      char c;
      file.read((char *)(&c), sizeof(c));
      timestamp_char[idx] = c;
    }
    timestamp_char[length] = '\0';
    std::string timestamp = timestamp_char;

    cv::Mat hist(bow_descriptors.rows, bow_descriptors.cols, type);
    for (int j = 0; j < cluster_size; j++) {
      float val = 0.0f;
      file.read((char *)(&val), sizeof(val));

      hist.ptr<float>(0)[j] = val;
    }

    double dist = cv::compareHist(hist, bow_descriptors, cv::HISTCMP_CORREL);
    if (dist > best_dist) {
      best_dist = dist;
      best_timestamp = timestamp;
    }

    delete[] timestamp_char;
  }

  return std::pair<std::string, double>(best_timestamp, best_dist);
}

void detect(const std::string &input_filename, const std::string &book_filename, cv::BOWImgDescriptorExtractor &bowDE,
            const cv::Ptr<cv::Feature2D> &detector, const std::string &training_dir) {
  cv::Mat mat_image = cv::imread(input_filename, cv::IMREAD_GRAYSCALE);

  if (mat_image.empty()) {
    std::cerr << "Empty image! " << input_filename << std::endl;
    return;
  }

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(mat_image, keypoints);

  cv::Mat bow_descritors;
  bowDE.compute(mat_image, keypoints, bow_descritors);

  std::pair<std::string, double> pair = compareHist(book_filename, bow_descritors);
  std::string best_timestamp = pair.first;
  double best_dist = pair.second;

  std::cout << "best_timestamp=" << best_timestamp << " ; best_dist=" << best_dist << std::endl;

#if DEBUG
  std::string training_result = training_dir + "/" + best_timestamp + ".jpeg";
  cv::Mat train_image = cv::imread(training_result, cv::IMREAD_GRAYSCALE);

  if (!train_image.empty()) {
    cv::Mat display_result;
    cv::hconcat(mat_image, train_image, display_result);
    cv::imshow("Query / Train images", display_result);
    cv::waitKey(30);
  }
#endif
}

int main(int argc, char *argv[]) {
  cv::CommandLineParser parser(argc, argv, keys);

  std::string pattern_glob = "";
  std::string input_image = "";
  if (parser.has("help")) {
    parser.printMessage();
    return EXIT_SUCCESS;
  } else if (parser.has("directory")) {
    pattern_glob = parser.get<std::string>("directory");
  } else if (parser.has("image")) {
    input_image = parser.get<std::string>("image");
  }

  if (pattern_glob.empty() && input_image.empty()) {
    std::cerr << "Images directory or input image are needed!" << std::endl;
    return EXIT_SUCCESS;
  }

  std::string detector_name = parser.get<std::string>("detector");
  std::string extractor_name = parser.get<std::string>("extractor");
  std::cout << "detector_name=" << detector_name << " ; extractor_name=" << extractor_name << std::endl;


  cv::Mat dictionary;
  std::string dictionary_name = parser.get<std::string>("dictionary");
  readBOWDictionary(dictionary_name, dictionary);
  std::cout << "dictionary=" << dictionary.rows << "x" << dictionary.cols << std::endl;

  std::string book_filename = parser.get<std::string>("book");

  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
  cv::Ptr<cv::Feature2D> detector, extractor;
  init_feature(detector_name, detector);
  init_feature(extractor_name, extractor);

  //BOW
  cv::BOWImgDescriptorExtractor bowDE(extractor, matcher);
  bowDE.setVocabulary(dictionary);


  std::vector<cv::String> image_filenames;
  if (!pattern_glob.empty()) {
    cv::String image_folder(pattern_glob);
    cv::glob(image_folder, image_filenames);
  } else {
    image_filenames.push_back(input_image);
  }

  std::string training_dir = parser.get<std::string>("train_dir");

  for (std::vector<cv::String>::const_iterator it_img = image_filenames.begin(); it_img != image_filenames.end(); ++it_img) {
    //Detect
    detect(*it_img, book_filename, bowDE, detector, training_dir);
  }


  return EXIT_SUCCESS;
}
#else
int main() {
  return EXIT_SUCCESS;
}
#endif
