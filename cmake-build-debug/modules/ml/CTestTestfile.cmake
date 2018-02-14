# CMake generated Testfile for 
# Source directory: /Users/chichunchen/workspace/opencv/modules/ml
# Build directory: /Users/chichunchen/workspace/opencv/cmake-build-debug/modules/ml
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_ml "/Users/chichunchen/workspace/opencv/cmake-build-debug/bin/opencv_test_ml" "--gtest_output=xml:opencv_test_ml.xml")
set_tests_properties(opencv_test_ml PROPERTIES  LABELS "Main;opencv_ml;Accuracy" WORKING_DIRECTORY "/Users/chichunchen/workspace/opencv/cmake-build-debug/test-reports/accuracy")
