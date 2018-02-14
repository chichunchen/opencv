# Install script for directory: /Users/chichunchen/workspace/opencv/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv" TYPE FILE FILES
    "/Users/chichunchen/workspace/opencv/include/opencv/cv.h"
    "/Users/chichunchen/workspace/opencv/include/opencv/cv.hpp"
    "/Users/chichunchen/workspace/opencv/include/opencv/cvaux.h"
    "/Users/chichunchen/workspace/opencv/include/opencv/cvaux.hpp"
    "/Users/chichunchen/workspace/opencv/include/opencv/cvwimage.h"
    "/Users/chichunchen/workspace/opencv/include/opencv/cxcore.h"
    "/Users/chichunchen/workspace/opencv/include/opencv/cxcore.hpp"
    "/Users/chichunchen/workspace/opencv/include/opencv/cxeigen.hpp"
    "/Users/chichunchen/workspace/opencv/include/opencv/cxmisc.h"
    "/Users/chichunchen/workspace/opencv/include/opencv/highgui.h"
    "/Users/chichunchen/workspace/opencv/include/opencv/ml.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv2" TYPE FILE FILES "/Users/chichunchen/workspace/opencv/include/opencv2/opencv.hpp")
endif()

