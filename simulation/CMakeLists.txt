cmake_minimum_required(VERSION 3.16...3.31)
project(ArucoSimulation VERSION 1.0
                        DESCRIPTION "Try to find boxes in a simulated environment"
                        LANGUAGES CXX)

# OpenCV depends on the deprecated FindCUDA for some reason
cmake_policy(SET CMP0146 OLD)

include(FetchContent)
FetchContent_Declare(GSL
    GIT_REPOSITORY "https://github.com/microsoft/GSL"
    GIT_TAG "v4.1.0"
    GIT_SHALLOW ON
)
FetchContent_MakeAvailable(GSL)

FetchContent_Declare(Eigen
    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    GIT_TAG "3.4.0"
    GIT_SHALLOW ON
)
FetchContent_MakeAvailable(Eigen)

find_package(OpenCV CONFIG REQUIRED core objdetect videoio highgui calib3d)

find_package(OGRE REQUIRED COMPONENTS Bites CONFIG)

# In case someone out there is still using a system with libm
find_library(MATH_LIBRARY m)

add_executable(generate_board generate_board.cpp)
target_compile_features(generate_board PUBLIC cxx_std_17)
set_target_properties(generate_board PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(generate_board PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(generate_board PRIVATE ${OpenCV_LIBS})

add_executable(positioner
    affine_rotation.h
    affine_rotation.cpp
    positioner.cpp
    visualization/visualizer.h
    visualization/visualizer.cpp)
target_compile_features(positioner PUBLIC cxx_std_17)
set_target_properties(positioner PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(positioner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(positioner PRIVATE ${OpenCV_LIBS} Microsoft.GSL::GSL OgreBites Eigen3::Eigen)
if(MATH_LIBRARY)
    target_link_libraries(positioner PUBLIC ${MATH_LIBRARY})
endif()
