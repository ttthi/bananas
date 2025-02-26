set(BOX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/boxes.json)
set(BOX_OUT_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/boxes)
set(OUT_BOXES ${BOX_OUT_PREFIX}/box_0.glb ${BOX_OUT_PREFIX}/box_0.sdf)

add_custom_command(
  OUTPUT ${OUT_BOXES}
  COMMAND gltf_exporter -o=boxes --sdf ${BOX_SOURCE}
  DEPENDS ${BOX_SOURCE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_custom_target(process-boxes ALL DEPENDS ${OUT_BOXES})

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/box_world.sdf
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/box_world.sdf
          ${CMAKE_CURRENT_BINARY_DIR}/box_world.sdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/box_world.sdf)
add_custom_target(copy-world ALL
                  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/box_world.sdf)
