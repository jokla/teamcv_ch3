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
  "{ help h       |      | print help message }"
  "{ directory d  |      | training images directory }"
  "{ detector     | SURF | detector name }"
  "{ extractor    | SURF | extractor name }"
  "{ cluster_size | 1500 | cluster size }"
};

std::string getName(const std::string& pathname) {
  if(pathname.size() > 0) {
    std::string convertedPathname = pathname;

    size_t index = convertedPathname.find_last_of("/");
    if(index != std::string::npos) {
      return convertedPathname.substr(index + 1);
    }

    return convertedPathname;
  }

  return "";
}

std::string getNameWE(const std::string& pathname) {
  std::string name = getName(pathname);
  size_t found = name.find_last_of(".");
  std::string name_we = name.substr(0, found);
  return name_we;
}

void saveBOWDictionary(const std::string &output_filename, const cv::Mat &dictionary) {
  std::ofstream file(output_filename.c_str(), std::ofstream::binary);

  if (!file.is_open()) {
    std::cerr << "Cannot I/O file: " << output_filename << std::endl;
    return;
  }

  if (dictionary.type() != CV_32F) {
    std::cerr << "Only CV_32F supported currently!" << std::endl;
    return;
  }

  //Write rows, cols, type
  int rows = dictionary.rows, cols = dictionary.cols, type = dictionary.type();
  file.write((char *)(&rows), sizeof(rows));
  file.write((char *)(&cols), sizeof(cols));
  file.write((char *)(&type), sizeof(type));

  for (int i = 0; i < dictionary.rows; i++) {
    for (int j = 0; j < dictionary.cols; j++) {
      float val = dictionary.ptr<float>(i)[j];
      file.write((char *)(&val), sizeof(val));
    }
  }
}

void train(const std::vector<cv::String> &input_training_data, cv::BOWKMeansTrainer &bowTrainer,
           cv::BOWImgDescriptorExtractor &bowDE, const cv::Ptr<cv::Feature2D> &detector,
           const cv::Ptr<cv::Feature2D> &extractor, const bool sameFeature,
           const std::string &output_dict_filename, const std::string &output_bow_book) {
#if DEBUG
  cv::namedWindow("Keypoints detected");
#endif

  int nb_valid_images = 0;
  for (std::vector<cv::String>::const_iterator it_filename = input_training_data.begin();
       it_filename != input_training_data.end(); ++it_filename) {
    cv::Mat mat_image = cv::imread(*it_filename, cv::IMREAD_GRAYSCALE);

    if (!mat_image.empty()) {
      nb_valid_images++;

      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      if (sameFeature) {
        extractor->detectAndCompute(mat_image, cv::noArray(), keypoints, descriptors);
      } else {
        detector->detect(mat_image, keypoints);
        extractor->compute(mat_image, keypoints, descriptors);
      }

#if DEBUG
      cv::Mat mat_image_display;
      cv::drawKeypoints(mat_image, keypoints, mat_image_display);
      cv::imshow("Keypoints detected", mat_image_display);
      cv::waitKey(100);
#endif

      bowTrainer.add(descriptors);
    }
  }

#if DEBUG
  cv::destroyWindow("Keypoints detected");
#endif

  //Cluster
  cv::Mat dictionary = bowTrainer.cluster();
  std::cout << "dictionary=" << dictionary.rows << "x" << dictionary.cols << std::endl;
  bowDE.setVocabulary(dictionary);
  saveBOWDictionary(output_dict_filename, dictionary);

  //Extract BOW histogram
  std::ofstream file_book(output_bow_book.c_str(), std::ofstream::binary);
  if (!file_book.is_open()) {
    std::cerr << "Cannot I/O file: " << output_bow_book << std::endl;
    return;
  }

  //Write book size and type
  file_book.write((char *)(&nb_valid_images), sizeof(nb_valid_images));
  file_book.write((char *)(&dictionary.rows), sizeof(dictionary.rows));
  int type = CV_32F;
  file_book.write((char *)(&type), sizeof(type));
#if DEBUG
  cv::Mat bowDescriptorsBook(0, 0, CV_32F);
#endif

  for (std::vector<cv::String>::const_iterator it_filename = input_training_data.begin();
       it_filename != input_training_data.end(); ++it_filename) {
    cv::Mat mat_image = cv::imread(*it_filename, cv::IMREAD_GRAYSCALE);

    if (!mat_image.empty()) {
      std::vector<cv::KeyPoint> keypoints;
      extractor->detect(mat_image, keypoints);

      cv::Mat bow_descritors;
      bowDE.compute(mat_image, keypoints, bow_descritors);

#if DEBUG
      bowDescriptorsBook.push_back(bow_descritors);
#endif

      std::string filename = getNameWE(*it_filename);
      //Write timestamp
      int filename_len = (int) filename.size();
      file_book.write((char *) (&filename_len), sizeof(filename_len));

      for(int cpt = 0; cpt < filename_len; cpt++) {
        file_book.write((char *) (&filename[(size_t)cpt]), sizeof(filename[(size_t)cpt]));
      }

      //Write histogram
      for (int i = 0; i < bow_descritors.rows; i++) {
        for (int j = 0; j < bow_descritors.cols; j++) {
          float val = bow_descritors.ptr<float>(i)[j];
          file_book.write((char *)(&val), sizeof(val));
        }
      }
    }
  }

#if DEBUG
  std::cout << "bowDescriptorsBook=" << bowDescriptorsBook.rows << "x" << bowDescriptorsBook.cols << std::endl;
#endif
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

int main(int argc, char *argv[]) {
  cv::CommandLineParser parser(argc, argv, keys);

  std::string pattern_glob = "";
  if (parser.has("help")) {
    parser.printMessage();
    return EXIT_SUCCESS;
  } else if (parser.has("directory")) {
    pattern_glob = parser.get<std::string>("directory");
  } else {
    std::cout << "Input training images directory is needed!" << std::endl;
    return EXIT_SUCCESS;
  }

  std::string detector_name = parser.get<std::string>("detector");
  std::string extractor_name = parser.get<std::string>("extractor");
  std::cout << "detector_name=" << detector_name << " ; extractor_name=" << extractor_name << std::endl;


  std::vector<cv::String> training_filenames;
  cv::String training_folder(pattern_glob);
  cv::glob(training_folder, training_filenames);

  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
  cv::Ptr<cv::Feature2D> detector, extractor;
  init_feature(detector_name, detector);
  init_feature(extractor_name, extractor);

  //Number of clusters
  int dictionarySize = parser.get<int>("cluster_size");
  std::cout << "dictionarySize=" << dictionarySize << std::endl;
  cv::TermCriteria tc(cv::TermCriteria::MAX_ITER, 10, 0.001);
  int retries = 1;
  int flags = cv::KMEANS_PP_CENTERS;
  cv::BOWKMeansTrainer bowTrainer(dictionarySize, tc, retries, flags);
  cv::BOWImgDescriptorExtractor bowDE(extractor, matcher);


  //Train
  train(training_filenames, bowTrainer, bowDE, detector, extractor,
        (detector_name == extractor_name), "dictionary.bin", "book.bin");


  return EXIT_SUCCESS;
}
#else
int main() {
  return EXIT_SUCCESS;
}
#endif
