cmake_minimum_required(VERSION 3.16)
# Fix behavior of CMAKE_CXX_STANDARD when targeting macOS.
if (POLICY CMP0025)
    cmake_policy(SET CMP0025 NEW)
endif ()
project(detection_tracking_control)

set(OpenCV_DIR "/usr/local/Cellar/opencv/4.5.0_5")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

#add_executable(detection_tracking_control common.hpp yolo.cpp)
add_executable(detection_tracking_control camshift.cpp)
#add_executable(detection_tracking_control pid.h pid.cpp)


set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_xfeatures2d opencv_dnn opencv_tracking)

# linking
target_link_libraries(detection_tracking_control ${OpenCV_LIBS})
