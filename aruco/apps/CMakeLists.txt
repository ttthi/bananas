add_executable(generate_board generate_board.cpp)
target_compile_features(generate_board PUBLIC cxx_std_17)
set_target_properties(generate_board PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(generate_board PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(generate_board PRIVATE ${OpenCV_LIBS})

add_executable(positioner positioner.cpp)
target_compile_features(positioner PUBLIC cxx_std_17)
set_target_properties(positioner PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(positioner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(positioner PRIVATE ${OpenCV_LIBS} aruco_detector
                                         aruco_visualizer)

add_executable(gltf_exporter gltf_exporter.cpp)
target_compile_features(gltf_exporter PUBLIC cxx_std_17)
set_target_properties(gltf_exporter PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(gltf_exporter PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  gltf_exporter
  PRIVATE ${OpenCV_LIBS} aruco_detector tinygltf Microsoft.GSL::GSL
          nlohmann_json::nlohmann_json tinyxml2::tinyxml2)
