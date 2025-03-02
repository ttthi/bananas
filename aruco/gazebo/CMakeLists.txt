set(BOARD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/boards.json)
set(BOARD_OUT_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/boards)
set(OUT_BOARDS ${BOARD_OUT_PREFIX}/board_0.glb ${BOARD_OUT_PREFIX}/board_0.sdf
               ${BOARD_OUT_PREFIX}/board_1.glb ${BOARD_OUT_PREFIX}/board_1.sdf)

add_custom_command(
  OUTPUT ${OUT_BOARDS}
  COMMAND gltf_exporter -o=boards --sdf ${BOARD_SOURCE}
  DEPENDS ${BOARD_SOURCE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_custom_target(process-boards ALL DEPENDS ${OUT_BOARDS})

set(COPIED_FILES box_world.sdf boards.json camera.json static_environment.json)
list(TRANSFORM COPIED_FILES PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/
                                    OUTPUT_VARIABLE COPY_SOURCE_FILES)
list(TRANSFORM COPIED_FILES PREPEND ${CMAKE_CURRENT_BINARY_DIR}/
                                    OUTPUT_VARIABLE COPY_DESTINATION_FILES)

add_custom_command(
  OUTPUT ${COPY_DESTINATION_FILES}
  COMMAND ${CMAKE_COMMAND} -E copy ${COPY_SOURCE_FILES}
          ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${COPY_SOURCE_FILES})
add_custom_target(copy-gazebo-config ALL DEPENDS ${COPY_DESTINATION_FILES})
