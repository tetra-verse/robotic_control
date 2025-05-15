get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(PRIVATE_SOURCES 
    logging.h
    logging.cpp
)

add_library(${TARGET_NAME} STATIC ${PRIVATE_SOURCES})
target_link_libraries(${TARGET_NAME} PUBLIC spdlog::spdlog_header_only fmt::fmt)
