cmake_minimum_required(VERSION 3.16...3.31)
project(ArucoSimulation VERSION 1.0
                        DESCRIPTION "Try to find boxes in a simulated environment"
                        LANGUAGES CXX)

# OpenCV depends on the deprecated FindCUDA for some reason
cmake_policy(SET CMP0146 OLD)

find_package(OpenCV CONFIG REQUIRED core objdetect imgcodecs highgui calib3d)

add_executable(generate_board generate_board.cpp)
target_compile_features(generate_board PUBLIC cxx_std_17)
set_target_properties(generate_board PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(generate_board PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(generate_board PRIVATE ${OpenCV_LIBS})

add_executable(positioner positioner.cpp)
target_compile_features(positioner PUBLIC cxx_std_17)
set_target_properties(positioner PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(positioner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(positioner PRIVATE ${OpenCV_LIBS})
