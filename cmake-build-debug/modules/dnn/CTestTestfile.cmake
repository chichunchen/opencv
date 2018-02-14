# CMake generated Testfile for 
# Source directory: /Users/chichunchen/workspace/opencv/modules/dnn
# Build directory: /Users/chichunchen/workspace/opencv/cmake-build-debug/modules/dnn
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_dnn "/Users/chichunchen/workspace/opencv/cmake-build-debug/bin/opencv_test_dnn" "--gtest_output=xml:opencv_test_dnn.xml")
set_tests_properties(opencv_test_dnn PROPERTIES  LABELS "Main;opencv_dnn;Accuracy" WORKING_DIRECTORY "/Users/chichunchen/workspace/opencv/cmake-build-debug/test-reports/accuracy")
add_test(opencv_perf_dnn "/Users/chichunchen/workspace/opencv/cmake-build-debug/bin/opencv_perf_dnn" "--gtest_output=xml:opencv_perf_dnn.xml")
set_tests_properties(opencv_perf_dnn PROPERTIES  LABELS "Main;opencv_dnn;Performance" WORKING_DIRECTORY "/Users/chichunchen/workspace/opencv/cmake-build-debug/test-reports/performance")
add_test(opencv_sanity_dnn "/Users/chichunchen/workspace/opencv/cmake-build-debug/bin/opencv_perf_dnn" "--gtest_output=xml:opencv_perf_dnn.xml" "--perf_min_samples=1" "--perf_force_samples=1" "--perf_verify_sanity")
set_tests_properties(opencv_sanity_dnn PROPERTIES  LABELS "Main;opencv_dnn;Sanity" WORKING_DIRECTORY "/Users/chichunchen/workspace/opencv/cmake-build-debug/test-reports/sanity")
