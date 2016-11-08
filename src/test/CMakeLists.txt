cmake_minimum_required(VERSION 2.8)

project( test_bow )

find_package( OpenCV 3 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( test_detect_bow test_detect_bow.cpp )
target_link_libraries( test_detect_bow ${OpenCV_LIBS} )

add_executable( test_train_bow test_train_bow.cpp )
target_link_libraries( test_train_bow ${OpenCV_LIBS} )
