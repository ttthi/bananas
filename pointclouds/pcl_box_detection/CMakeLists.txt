cmake_minimum_required(VERSION 3.16...3.26)
project(
  BOX_DETECTION
  VERSION 0.1
  LANGUAGES C CXX)

find_package(
  PCL 1.3
  COMPONENTS common filters segmentation visualization CONFIG
  REQUIRED)
find_package(OpenCV CONFIG REQUIRED core imgproc)
find_package(Eigen3 3.3 NO_MODULE REQUIRED)
find_package(depthai CONFIG)

# TODO: Why is this needed? What does this line do?
add_library(JsonCpp::JsonCpp INTERFACE IMPORTED)

function(add_box_detection_exe NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_compile_features(${NAME} PUBLIC cxx_std_20)
  set_target_properties(${NAME} PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(${NAME} PRIVATE -Wall -Wextra -Wpedantic)

  target_include_directories(${NAME} PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_directories(${NAME} PRIVATE ${PCL_LIBRARY_DIRS})
  target_compile_definitions(${NAME} PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(${NAME} PRIVATE ${PCL_LIBRARIES})
endfunction()

add_box_detection_exe(box_detection)
add_box_detection_exe(box_detection2)

if(depthai_FOUND)
  message(STATUS "Building DepthAI support")
  add_executable(store_clouds store_clouds.cpp)
  target_compile_features(store_clouds PUBLIC cxx_std_20)
  set_target_properties(store_clouds PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(store_clouds PRIVATE -Wall -Wextra -Wpedantic)

  target_include_directories(store_clouds PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_directories(store_clouds PRIVATE ${PCL_LIBRARY_DIRS}
                          ${OpenCV_LIBRARY_DIRS})
  target_compile_definitions(store_clouds PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(store_clouds PRIVATE depthai::all ${PCL_LIBRARIES}
                                             ${OpenCV_LIBS})

  add_executable(test_imu test_imu.cpp)
  target_compile_features(test_imu PUBLIC cxx_std_20)
  set_target_properties(test_imu PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(test_imu PRIVATE -Wall -Wextra -Wpedantic)

  target_link_libraries(test_imu PRIVATE depthai::core Eigen3::Eigen)
else()
  message(STATUS "DepthAI library not found, not including DepthAI support")
endif()
