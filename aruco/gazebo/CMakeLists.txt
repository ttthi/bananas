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

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/box_world.sdf
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/box_world.sdf
          ${CMAKE_CURRENT_BINARY_DIR}/box_world.sdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/box_world.sdf)
add_custom_target(copy-world ALL
                  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/box_world.sdf)
