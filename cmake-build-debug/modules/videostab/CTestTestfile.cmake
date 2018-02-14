# CMake generated Testfile for 
# Source directory: /Users/chichunchen/workspace/opencv/modules/videostab
# Build directory: /Users/chichunchen/workspace/opencv/cmake-build-debug/modules/videostab
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_videostab "/Users/chichunchen/workspace/opencv/cmake-build-debug/bin/opencv_test_videostab" "--gtest_output=xml:opencv_test_videostab.xml")
set_tests_properties(opencv_test_videostab PROPERTIES  LABELS "Main;opencv_videostab;Accuracy" WORKING_DIRECTORY "/Users/chichunchen/workspace/opencv/cmake-build-debug/test-reports/accuracy")
