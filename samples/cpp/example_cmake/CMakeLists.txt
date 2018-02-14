# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(opencv_example_project)

set("OpenCV_DIR" "/Users/chichunchen/workspace/opencv/build")
set(CMAKE_CXX_FLAGS "-std=gnu++11 -g")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()
include_directories(.)

# Declare the executable target built from your sources
set(YOLO_SOURCES yolo.cpp objdet.cpp objdet.hpp)
add_executable(yolo_objdet ${YOLO_SOURCES})
set(BM_SOURCES bm.cpp objdet.cpp objdet.hpp)
add_executable(bm ${BM_SOURCES})

# Link your application with OpenCV libraries
target_link_libraries(yolo_objdet ${OpenCV_LIBS})
target_link_libraries(bm ${OpenCV_LIBS})
