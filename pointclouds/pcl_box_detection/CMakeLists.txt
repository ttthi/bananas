cmake_minimum_required(VERSION 3.16...3.26)
project(BOX_DETECTION VERSION 0.1
                      LANGUAGES C CXX)

find_package(PCL 1.3 COMPONENTS common filters segmentation visualization CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED core imgproc)
find_package(depthai CONFIG REQUIRED)

# TODO: Why is this needed? What does this line do?
add_library(JsonCpp::JsonCpp INTERFACE IMPORTED)

add_executable(box_detection box_detection2.cpp)
target_compile_features(box_detection PUBLIC cxx_std_20)
set_target_properties(box_detection PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(box_detection PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(box_detection PRIVATE
 ${PCL_INCLUDE_DIRS})
target_link_directories(box_detection PRIVATE
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS})
target_compile_definitions(box_detection PRIVATE
  ${PCL_DEFINITIONS})
target_link_libraries(box_detection PRIVATE
  depthai::all
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(store_clouds store_clouds.cpp)
target_compile_features(store_clouds PUBLIC cxx_std_20)
set_target_properties(store_clouds PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(store_clouds PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(store_clouds PRIVATE
 ${PCL_INCLUDE_DIRS})
target_link_directories(store_clouds PRIVATE
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS})
target_compile_definitions(store_clouds PRIVATE
  ${PCL_DEFINITIONS})
target_link_libraries(store_clouds PRIVATE
  depthai::all
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS})
