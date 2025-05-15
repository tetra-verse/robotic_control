set(TARGET_NAME CoreSDK)
add_library(${TARGET_NAME} INTERFACE)

set(RUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_include_directories(${TARGET_NAME}
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_directories(${TARGET_NAME} INTERFACE ${RUNTIME_DIR})
target_link_libraries(${TARGET_NAME} INTERFACE xCoreSDK xMateModeld eigen)


add_custom_target(
  copy_alog_runtime_files ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${RUNTIME_DIR}
          ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  COMMENT "Copying ALGO runtime files to output directory")

add_dependencies(${TARGET_NAME} copy_alog_runtime_files)
