add_library(aruco_detector
  affine_rotation.cpp
  box_board.cpp
  world.cpp
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/affine_rotation.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/box_board.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/world.h"
)

target_include_directories(aruco_detector PUBLIC "${PROJECT_SOURCE_DIR}/include")

target_compile_features(aruco_detector PUBLIC cxx_std_17)
set_target_properties(aruco_detector PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(aruco_detector PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(aruco_detector PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
target_link_libraries(aruco_detector PRIVATE Microsoft.GSL::GSL)
if(MATH_LIBRARY)
    target_link_libraries(aruco_detector PRIVATE ${MATH_LIBRARY})
endif()

add_subdirectory(visualization)
